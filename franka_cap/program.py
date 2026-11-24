"""Interpret a bounded run(api) program without eval, exec, imports or OS access."""
import ast
import math
import time

TOOLS={'describe','get_robot_state','get_observation','localize_pixel','detect_color','offset_pose','solve_ik','plan_motion','move_to_pose','set_gripper','measure_region','grasp','step_toward'}
class ProgramPaused(Exception):
    pass


class Program:
    def __init__(self,api,max_calls=32,timeout_s=180,allowed_tools=None,should_pause=lambda: False):
        self.allowed_tools = TOOLS if allowed_tools is None else set(allowed_tools)
        self.should_pause = should_pause
        self.api=api;self.max_calls=max_calls;self.timeout_s=timeout_s;self.calls=0;self.steps=0;self.variables={}

    def validate(self,source):
        if not isinstance(source,str) or len(source.encode("utf-8"))>20000:raise ValueError('Program size limit')
        tree=ast.parse(source)
        if len(tree.body)!=1 or not isinstance(tree.body[0],ast.FunctionDef):raise ValueError('Expected only def run(api)')
        fn=tree.body[0]
        if fn.name!='run' or fn.decorator_list or fn.returns or fn.args.posonlyargs or fn.args.kwonlyargs or fn.args.vararg or fn.args.kwarg or fn.args.defaults or len(fn.args.args)!=1 or fn.args.args[0].arg!='api' or fn.args.args[0].annotation:
            raise ValueError('Expected undecorated def run(api)')
        nodes=list(ast.walk(tree))
        if len(nodes)>1500:raise ValueError('AST size limit')
        allowed=(ast.Module,ast.FunctionDef,ast.arguments,ast.arg,ast.Return,ast.Expr,ast.Assign,ast.Name,ast.Load,ast.Store,
                 ast.Constant,ast.List,ast.Tuple,ast.Dict,ast.Subscript,ast.Call,ast.Attribute,ast.keyword,ast.UnaryOp,ast.USub,ast.UAdd,
                 ast.BinOp,ast.Add,ast.Sub,ast.Mult,ast.Div,ast.Compare,ast.Eq,ast.NotEq,ast.Lt,ast.LtE,ast.Gt,ast.GtE,
                 ast.If,ast.For,ast.Pass)
        for node in nodes:
            if not isinstance(node,allowed):raise ValueError('Unsupported syntax: '+type(node).__name__)
            if isinstance(node,ast.FunctionDef) and node is not fn:raise ValueError('Nested functions forbidden')
            if isinstance(node,ast.Name) and node.id.startswith('_'):raise ValueError('Private names forbidden')
            if isinstance(node,ast.Constant) and (type(node.value) not in (str,int,float,bool,type(None)) or (isinstance(node.value,str) and len(node.value)>10000)):
                raise ValueError('Unsupported constant')
            if isinstance(node,ast.Constant) and type(node.value) in (float,int) and (not math.isfinite(node.value) or abs(node.value)>1e9):
                raise ValueError('Bounded finite constants required')
            if isinstance(node,ast.Attribute) and (not isinstance(node.value,ast.Name) or node.value.id!='api' or node.attr not in self.allowed_tools):
                raise ValueError('Only documented api methods are accessible')
            if isinstance(node,ast.Call):
                if any(k.arg is None for k in node.keywords):raise ValueError('No argument unpacking')
                if isinstance(node.func,ast.Name):
                    if node.func.id!='range' or len(node.args)!=1 or node.keywords or not isinstance(node.args[0],ast.Constant) or type(node.args[0].value) is not int or not 0<=node.args[0].value<=20:
                        raise ValueError('Only literal range(0..20) is allowed')
                elif not isinstance(node.func,ast.Attribute):raise ValueError('Only api calls are allowed')
            if isinstance(node,ast.Assign) and (len(node.targets)!=1 or not isinstance(node.targets[0],ast.Name) or node.targets[0].id in ('api','range')):
                raise ValueError('Only simple variable assignments')
            if isinstance(node,ast.For) and (not isinstance(node.target,ast.Name) or node.target.id in ('api','range') or node.orelse or not isinstance(node.iter,ast.Call) or not isinstance(node.iter.func,ast.Name) or node.iter.func.id!='range'):
                raise ValueError('Only bounded range loops are supported')
            if isinstance(node,ast.Dict) and any(k is None for k in node.keys):raise ValueError('No dictionary unpacking')
        return fn

    def run(self,source):
        fn=self.validate(source)  # validate the entire program before any hardware call
        self.started=time.monotonic();self.calls=0;self.steps=0;self.variables={}
        motion=getattr(self.api,"_motion",None)
        before=getattr(motion,"calls",0)
        returned,value=self.block(fn.body)
        after=getattr(motion,"calls",0)
        motion_sent=type(before) is int and type(after) is int and after>before
        return {'returned':value,'api_calls':self.calls,'executed_motion':motion_sent,'program_contract':'bounded_run_api_v1'}

    def tick(self):
        self.steps+=1
        if self.steps>10000 or time.monotonic()-self.started>self.timeout_s:raise ValueError('Program budget exceeded')

    def block(self,statements):
        for node in statements:
            self.tick()
            if isinstance(node,ast.Return):return True,self.value(node.value) if node.value else None
            if isinstance(node,ast.Assign):self.variables[node.targets[0].id]=self.value(node.value)
            elif isinstance(node,ast.Expr):self.value(node.value)
            elif isinstance(node,ast.If):
                returned,value=self.block(node.body if self.value(node.test) else node.orelse)
                if returned:return True,value
            elif isinstance(node,ast.For):
                for i in self.value(node.iter):
                    self.variables[node.target.id]=i
                    returned,value=self.block(node.body)
                    if returned:return True,value
            elif not isinstance(node,ast.Pass):raise ValueError('Unsupported statement')
        return False,None

    def value(self,node):
        self.tick()
        if isinstance(node,ast.Constant):return node.value
        if isinstance(node,ast.Name):return self.variables[node.id]
        if isinstance(node,(ast.List,ast.Tuple)):return [self.value(n) for n in node.elts]
        if isinstance(node,ast.Dict):return {self.value(k):self.value(v) for k,v in zip(node.keys,node.values)}
        if isinstance(node,ast.Subscript):return self.value(node.value)[self.value(node.slice)]
        if isinstance(node,ast.UnaryOp):
            value=self.number(self.value(node.operand));return -value if isinstance(node.op,ast.USub) else value
        if isinstance(node,ast.BinOp):
            a=self.number(self.value(node.left));b=self.number(self.value(node.right))
            if isinstance(node.op,ast.Add):return a+b
            if isinstance(node.op,ast.Sub):return a-b
            if isinstance(node.op,ast.Mult):return a*b
            return a/b
        if isinstance(node,ast.Compare):
            a=self.value(node.left)
            for op,right in zip(node.ops,node.comparators):
                b=self.value(right)
                if isinstance(op,ast.Eq):ok=a==b
                elif isinstance(op,ast.NotEq):ok=a!=b
                elif isinstance(op,ast.Lt):ok=a<b
                elif isinstance(op,ast.LtE):ok=a<=b
                elif isinstance(op,ast.Gt):ok=a>b
                else:ok=a>=b
                if not ok:return False
                a=b
            return True
        if isinstance(node,ast.Call):
            if isinstance(node.func,ast.Name):return range(node.args[0].value)
            if self.should_pause():raise ProgramPaused()
            self.calls+=1
            if self.calls>self.max_calls:raise ValueError('API call budget exceeded')
            args=[self.value(v) for v in node.args];kwargs={k.arg:self.value(k.value) for k in node.keywords}
            return getattr(self.api,node.func.attr)(*args,**kwargs)
        raise ValueError('Unsupported expression')

    @staticmethod
    def number(value):
        if type(value) not in (int,float) or abs(value)>1e9:raise ValueError('Bounded numeric operands required')
        return value
