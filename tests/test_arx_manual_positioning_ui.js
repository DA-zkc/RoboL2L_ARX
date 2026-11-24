// Isolated DOM and fetch fixture: no live Runtime requests.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const source = fs.readFileSync(path.join(__dirname, '../robol2l_deploy/web/manual-positioning.js'), 'utf8');
const html = fs.readFileSync(path.join(__dirname, '../robol2l_deploy/web/index.html'), 'utf8');
assert.equal((html.match(/data-manual-positioning/g) || []).length, 2);
function panel() {
  const controls = new Map();
  const el = key => {
    if (!controls.has(key)) controls.set(key, {value: 'left', hidden: true, disabled: true,
      dataset: key.includes('recheck-hold') ? {recover: 'recheck-hold'} : {recover: 'acknowledge-parked'}});
    return controls.get(key);
  };
  return {hidden: true, dataset: {}, querySelector: el, querySelectorAll: () =>
    [el('[data-recover="recheck-hold"]'), el('[data-recover="acknowledge-parked"]')]};
}
async function fixture(backend = 'arx_x5_2023_dual') {
  const panels = [panel(), panel()], posts = [], timers = [];
  let state = {teach: {phase: 'idle', active: false}}, fail = false, confirmations = 0;
  const context = vm.createContext({document: {querySelectorAll: () => panels},
    confirm: () => { confirmations++; return true; },
    setInterval: callback => timers.push(callback),
    fetch: async (url, opts = {}) => {
      if (opts.method === 'POST') {
        const body = JSON.parse(opts.body);
        posts.push({url, body, headers: opts.headers});
        if (url.endsWith('/start')) state = {teach: {phase: 'countdown', side: body.side, active: true, countdown_s: 10}};
        if (url.endsWith('/hold')) state = {teach: {phase: 'held', side: state.teach.side, active: false}};
        return {ok: true, json: async () => ({})};
      }
      if (fail) throw Error('offline');
      return {ok: true, json: async () => url.endsWith('/session') ? {backend, session_id: 'fixture'} : state};
    }
  });
  vm.runInContext(source, context);
  await new Promise(resolve => setImmediate(resolve));
  return {panels, posts, timers, setState: s => state = s, fail: f => fail = f, confirms: () => confirmations};
}
(async () => {
  const f = await fixture(), [controls, experiment] = f.panels;
  assert.equal(controls.hidden, false); assert.equal(experiment.hidden, false);
  assert.equal(controls.querySelector('[data-start]').disabled, false);
  experiment.querySelector('select').value = 'right';
  await experiment.querySelector('[data-start]').onclick();
  assert.equal(f.posts.length, 1); assert.equal(f.posts[0].body.side, 'right');
  assert.equal(f.posts[0].headers['X-Session-Id'], 'fixture');
  assert.equal(f.confirms(), 0); // Start is exactly one click.
  assert.match(controls.querySelector('[data-state]').textContent, /10 s/);
  assert.equal(controls.querySelector('select').value, 'right');
  assert.match(controls.querySelector('[data-hold]').textContent, /Cancel/);
  f.setState({teach: {phase:'drag', side:'right', active:true}});
  for (let i=0; i<70; i++) await f.timers[0]();
  assert.equal(f.posts.length, 1); // Polling never auto-holds or restarts.
  await controls.querySelector('[data-hold]').onclick();
  assert.equal(f.posts.length, 2); assert.match(f.posts[1].url, /\/hold$/);
  f.setState({teach: {phase:'error', side:'left', active:false, can_recheck_hold:true}, blocked_reason:'station_faulted'});
  await f.timers[0]();
  assert.equal(experiment.querySelector('[data-start]').disabled, true);
  const recover = controls.querySelector('[data-recover="recheck-hold"]');
  assert.equal(recover.hidden, false); assert.equal(recover.disabled, false);
  await recover.onclick(); await new Promise(resolve => setImmediate(resolve));
  assert.equal(f.confirms(), 1); assert.equal(f.posts[2].body.confirmed, true);
  f.fail(true); await f.timers[0]();
  assert.equal(recover.disabled, true);
  assert.equal(f.posts.length, 3);
  const other = await fixture('mock');
  assert(other.panels.every(p => p.hidden)); assert.equal(other.posts.length, 0);
  console.log('Shared manual-positioning UI: one-click countdown, persistent drag, hold and explicit recovery passed');
})().catch(error => { console.error(error); process.exitCode = 1; });
