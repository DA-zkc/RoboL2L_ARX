# Historical evaluator only. Active CaP uses cap/worker.py and ASPIRE ProgramExecutor.
"""Bounded Python AST evaluator. No eval/exec, host methods, files or network.

Programs use the RoboL2L top-level format and public data types. The intentionally
small syntax subset is listed in validate(); unsupported syntax fails before tools.
"""
import ast
from collections.abc import Mapping
from dataclasses import dataclass, fields, is_dataclass
import math
import operator
import time
import numpy as np


class Halt(BaseException):
    """Host budget/pause termination cannot be caught by a generated program."""


class Returned(BaseException):
    def __init__(self, value): self.value = value


@dataclass
class Function:
    node: ast.FunctionDef
    environment: dict
    defaults: tuple


BINARY = {ast.Add: operator.add, ast.Sub: operator.sub, ast.Mult: operator.mul,
          ast.Div: operator.truediv, ast.FloorDiv: operator.floordiv, ast.Mod: operator.mod,
          ast.Pow: operator.pow}
COMPARE = {ast.Eq: operator.eq, ast.NotEq: operator.ne, ast.Lt: operator.lt,
           ast.LtE: operator.le, ast.Gt: operator.gt, ast.GtE: operator.ge,
           ast.Is: operator.is_, ast.IsNot: operator.is_not,
           ast.In: lambda a,b: a in b, ast.NotIn: lambda a,b: a not in b}
MATH = {name: getattr(math, name) for name in (
    'pi','e','sin','cos','tan','asin','acos','atan','atan2','sqrt','hypot',
    'radians','degrees','floor','ceil','fabs','isfinite','isclose')}
BUILTINS = {name: value for name, value in dict(abs=abs, bool=bool, float=float,
    int=int, len=len, max=max, min=min, round=round, str=str, sum=sum,
    all=all, any=any, list=list, tuple=tuple, dict=dict, enumerate=enumerate,
    zip=zip, range=range, sorted=sorted, reversed=reversed,
    Exception=Exception, ValueError=ValueError, TypeError=TypeError).items()}


def validate(source):
    if not isinstance(source, str) or not source.strip() or len(source.encode()) > 32000:
        raise ValueError('source must contain 1..32000 bytes')
    tree = ast.parse(source, filename='<cap-program>')
    nodes = list(ast.walk(tree))
    if len(nodes) > 3000: raise ValueError('AST size limit')
    allowed = (ast.Module, ast.Import, ast.alias, ast.Assign, ast.AugAssign, ast.Name,
        ast.Load, ast.Store, ast.Constant, ast.Expr, ast.List, ast.Tuple, ast.Dict,
        ast.Subscript, ast.Slice, ast.Attribute, ast.Call, ast.keyword, ast.If,
        ast.IfExp, ast.For, ast.While, ast.Break, ast.Continue, ast.Pass,
        ast.FunctionDef, ast.arguments, ast.arg, ast.Return, ast.Assert, ast.Raise,
        ast.Try, ast.ExceptHandler, ast.BinOp, ast.UnaryOp, ast.BoolOp, ast.Compare,
        ast.And, ast.Or, ast.Not, ast.USub, ast.UAdd, ast.ListComp, ast.GeneratorExp,
        ast.comprehension, *BINARY.keys(), *COMPARE.keys())
    for node in nodes:
        problem = None
        if not isinstance(node, allowed): problem = f'{type(node).__name__} is not supported'
        if isinstance(node, ast.Import) and any(a.name != 'math' or a.asname for a in node.names): problem = 'only import math is allowed'
        if isinstance(node, ast.Name) and node.id.startswith('_'): problem = 'private names are forbidden'
        if isinstance(node, ast.Attribute) and (node.attr.startswith('_') or isinstance(node.ctx, ast.Store)): problem = 'private or writable attributes are forbidden'
        if isinstance(node, ast.Subscript) and isinstance(node.ctx, ast.Store): problem = 'subscript mutation is not supported'
        if isinstance(node, ast.FunctionDef) and (node.decorator_list or node.returns or node.name.startswith('_')): problem = 'decorated/annotated/private functions are forbidden'
        if isinstance(node, ast.arguments) and (node.posonlyargs or node.kwonlyargs or node.vararg or node.kwarg): problem = 'use simple function parameters with optional defaults'
        if isinstance(node, ast.arg) and (node.annotation or node.arg.startswith('_')): problem = 'invalid function parameter'
        if isinstance(node, ast.Dict) and any(k is None for k in node.keys): problem = 'dictionary unpacking is not supported'
        if isinstance(node, ast.keyword) and node.arg is None: problem = 'keyword unpacking is not supported'
        if isinstance(node, ast.comprehension) and node.is_async: problem = 'async is not supported'
        if isinstance(node, ast.Raise) and node.cause: problem = 'raise from is not supported'
        if problem:
            error = ValueError(problem); error.line = getattr(node, 'lineno', None); raise error
    compile(tree, "<cap-program>", "exec")  # Syntax checks only; this code object is never executed.
    return tree


class Break(BaseException): pass
class Continue(BaseException): pass


class Evaluator:
    def __init__(self, namespace, *, max_steps=10000, timeout_s=30):
        self.environment = dict(BUILTINS, **namespace)
        self.allowed = list(BUILTINS.values()) + list(namespace.values()) + list(MATH.values())
        self.max_steps = max_steps; self.deadline = time.monotonic()+timeout_s
        self.steps = 0; self.line = None; self.depth = 0

    def tick(self, node=None):
        self.steps += 1
        if node is not None: self.line = getattr(node, 'lineno', self.line)
        if self.steps > self.max_steps or time.monotonic() > self.deadline: raise Halt('program_budget_exceeded')

    def sequence(self, value):
        for i, item in enumerate(value):
            self.tick()
            if i >= 4096: raise Halt('sequence_budget_exceeded')
            yield item

    def bind(self, target, value, env):
        if isinstance(target, ast.Name): env[target.id] = value
        elif isinstance(target, (ast.List, ast.Tuple)):
            values = list(self.sequence(value))
            if len(values) != len(target.elts): raise ValueError('unpack size mismatch')
            for t,v in zip(target.elts, values): self.bind(t,v,env)
        else: raise ValueError('assignment target is not supported')

    def block(self, statements, env):
        for node in statements:
            self.tick(node)
            if isinstance(node, ast.Import): env['math'] = MATH
            elif isinstance(node, ast.Assign):
                value = self.value(node.value, env)
                for target in node.targets: self.bind(target,value,env)
            elif isinstance(node, ast.AugAssign): self.bind(node.target,self.binary(node.op,self.value(node.target,env),self.value(node.value,env)),env)
            elif isinstance(node, ast.Expr): self.value(node.value,env)
            elif isinstance(node, ast.If): self.block(node.body if self.value(node.test,env) else node.orelse,env)
            elif isinstance(node, (ast.For,ast.While)):
                items = self.sequence(self.value(node.iter,env)) if isinstance(node,ast.For) else iter(int,1)
                broken = False
                for item in items:
                    self.tick(node)
                    if isinstance(node,ast.While) and not self.value(node.test,env): break
                    if isinstance(node,ast.For): self.bind(node.target,item,env)
                    try: self.block(node.body,env)
                    except Continue: continue
                    except Break: broken=True; break
                if not broken: self.block(node.orelse,env)
            elif isinstance(node,ast.Break): raise Break()
            elif isinstance(node,ast.Continue): raise Continue()
            elif isinstance(node,ast.FunctionDef): env[node.name]=Function(node,env,tuple(self.value(d,env) for d in node.args.defaults))
            elif isinstance(node,ast.Return): raise Returned(self.value(node.value,env) if node.value else None)
            elif isinstance(node,ast.Assert):
                if not self.value(node.test,env): raise AssertionError(self.value(node.msg,env) if node.msg else 'assertion failed')
            elif isinstance(node,ast.Raise):
                error=self.value(node.exc,env) if node.exc else ValueError('bare raise is not supported')
                if not isinstance(error,Exception): raise TypeError('raise requires an exception')
                raise error
            elif isinstance(node,ast.Try):
                try: self.block(node.body,env)
                except Exception as error:
                    for handler in node.handlers:
                        expected=self.value(handler.type,env) if handler.type else Exception
                        if expected not in (Exception,ValueError,TypeError): raise ValueError('unsupported exception handler')
                        if isinstance(error,expected):
                            if handler.name:env[handler.name]=str(error)
                            self.block(handler.body,env);break
                    else:raise
                else:self.block(node.orelse,env)
                finally:self.block(node.finalbody,env)
            elif not isinstance(node,ast.Pass): raise ValueError('unsupported statement')

    def binary(self, op, a, b):
        # Bound allocations before invoking Python/NumPy operators.
        for value in (a,b):
            if isinstance(value,(str,list,tuple,np.ndarray)) and len(value)>4096: raise Halt('operand_size_limit')
            if isinstance(value,int) and value.bit_length()>4096: raise Halt('integer_size_limit')
        if isinstance(op,ast.Mult):
            for seq,n in ((a,b),(b,a)):
                if isinstance(seq,(str,list,tuple)) and isinstance(n,int) and len(seq)*max(n,0)>4096: raise Halt('allocation_limit')
        if isinstance(op,ast.Pow) and (not isinstance(b,(int,float)) or abs(b)>16): raise Halt('exponent_limit')
        result = BINARY[type(op)](a,b)
        if isinstance(result,int) and result.bit_length()>4096:raise Halt('integer_size_limit')
        return result

    def value(self, node, env):
        self.tick(node)
        if isinstance(node,ast.Constant): return node.value
        if isinstance(node,ast.Name):
            if node.id not in env: raise NameError(f'name {node.id!r} is not defined')
            return env[node.id]
        if isinstance(node,(ast.List,ast.Tuple)):
            items = [self.value(n,env) for n in node.elts]
            return tuple(items) if isinstance(node,ast.Tuple) else items
        if isinstance(node,ast.Dict): return {self.value(k,env):self.value(v,env) for k,v in zip(node.keys,node.values)}
        if isinstance(node,ast.Slice): return slice(*(self.value(n,env) if n else None for n in (node.lower,node.upper,node.step)))
        if isinstance(node,ast.Subscript): return self.value(node.value,env)[self.value(node.slice,env)]
        if isinstance(node,ast.Attribute):
            obj=self.value(node.value,env)
            if obj is MATH and node.attr in MATH:return MATH[node.attr]
            names={f.name for f in fields(obj)} if is_dataclass(obj) and not isinstance(obj,type) else set()
            names |= {'arms','position_m','frame'} if is_dataclass(obj) and not isinstance(obj,type) else set()
            if node.attr in names:return getattr(obj,node.attr)
            if isinstance(obj,np.ndarray) and node.attr in ('shape','ndim','size'):return getattr(obj,node.attr)
            raise AttributeError(f'attribute {node.attr!r} is not public')
        if isinstance(node,ast.BinOp):return self.binary(node.op,self.value(node.left,env),self.value(node.right,env))
        if isinstance(node,ast.UnaryOp):
            value=self.value(node.operand,env)
            return not value if isinstance(node.op,ast.Not) else -value if isinstance(node.op,ast.USub) else +value
        if isinstance(node,ast.BoolOp):
            value=None
            for expr in node.values:
                value=self.value(expr,env)
                if isinstance(node.op,ast.And) and not value or isinstance(node.op,ast.Or) and value:break
            return value
        if isinstance(node,ast.Compare):
            a=self.value(node.left,env)
            for op,right in zip(node.ops,node.comparators):
                b=self.value(right,env)
                if not COMPARE[type(op)](a,b):return False
                a=b
            return True
        if isinstance(node,ast.IfExp):return self.value(node.body if self.value(node.test,env) else node.orelse,env)
        if isinstance(node,(ast.ListComp,ast.GeneratorExp)):
            result=[]
            def collect(index,scope):
                if index==len(node.generators):
                    if len(result)>=4096:raise Halt('comprehension_limit')
                    result.append(self.value(node.elt,scope));return
                gen=node.generators[index]
                for item in self.sequence(self.value(gen.iter,scope)):
                    local=dict(scope);self.bind(gen.target,item,local)
                    if all(self.value(test,local) for test in gen.ifs):collect(index+1,local)
            collect(0,dict(env));return result
        if isinstance(node,ast.Call):
            fn=self.value(node.func,env);args=[self.value(n,env) for n in node.args];kwargs={k.arg:self.value(k.value,env) for k in node.keywords}
            self.line=node.lineno
            if isinstance(fn,Function):
                self.depth+=1
                try:
                    if self.depth>32:raise Halt('recursion_limit')
                    names=[arg.arg for arg in fn.node.args.args]
                    if len(args)>len(names) or set(kwargs)-set(names) or set(names[:len(args)])&set(kwargs):raise TypeError('invalid function arguments')
                    values=dict(zip(names[-len(fn.defaults):],fn.defaults)) if fn.defaults else {}
                    values.update(zip(names,args));values.update(kwargs)
                    if set(values)!=set(names):raise TypeError('missing function argument')
                    local=dict(fn.environment,**values)
                    try:self.block(fn.node.body,local)
                    except Returned as result:return result.value
                    return None
                finally:self.depth-=1
            if not any(fn is allowed for allowed in self.allowed):raise TypeError('call target is not allowed')
            if fn is range:
                value=range(*args,**kwargs)
                if len(value)>4096:raise Halt('range_limit')
                return value
            if fn in (list,tuple,sum,all,any,sorted,reversed) and args:
                args[0]=list(self.sequence(args[0]))
            if fn in (int,float,str) and args and isinstance(args[0],(str,int)) and len(str(args[0]))>4096:raise Halt('conversion_limit')
            return fn(*args,**kwargs)
        raise ValueError(f'unsupported expression {type(node).__name__}')

    def run(self, tree):
        self.block(tree.body,self.environment)
        return self.environment.get('result')
