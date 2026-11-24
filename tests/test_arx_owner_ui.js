// Offline DOM/fetch fixture: never contacts the running launcher or Runtime.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const path = require('node:path');
const source = fs.readFileSync(path.join(__dirname, '../robol2l_deploy/web/launcher.js'), 'utf8');
async function fixture(resident, updatedLauncher) {
  const elements = new Map(), posts = [];
  const element = id => {
    if (!elements.has(id)) elements.set(id, {checked: false, value: '', hidden: false, disabled: false});
    return elements.get(id);
  };
  const state = {running: true, session: {backend: 'arx_x5_2023_dual', url: 'http://fixture'},
    runtime: {busy: false, cap_running: false}};
  if (resident) state.runtime.hardware_lifecycle = {mode: 'resident_arx_owner_v1'};
  if (updatedLauncher) state.lifecycle_protocol = 'resident_arx_owner_v1';
  const context = vm.createContext({document: {getElementById: element, querySelectorAll: () => []},
    window: {location: {assign() {}}}, setInterval() {},
    fetch: async (url, options = {}) => {
      if (options.method === 'POST') posts.push({url, body: JSON.parse(options.body)});
      return {ok: true, json: async () => state};
    }});
  vm.runInContext(source, context);
  await new Promise(resolve => setImmediate(resolve));
  return {element, posts, state};
}
(async () => {
  const old = await fixture(false, false);
  assert.equal(old.element('reload-session').disabled, true);
  assert.equal(old.element('stop-session').disabled, true);
  // Even programmatically invoking a disabled click must not reach the old API.
  await old.element('reload-session').onclick();
  await old.element('stop-session').onclick();
  old.element('hardware-supported').checked = true;
  await old.element('maintenance-reload').onclick();
  old.element('backend-choice').value = 'franky';
  await old.element('connect-real').onclick();
  assert.equal(old.posts.length, 0);

  const migration = await fixture(false, true);
  migration.element('hardware-supported').checked = true;
  await migration.element('maintenance-reload').onclick();
  assert.deepEqual(migration.posts, [{url: '/api/launcher/maintenance/reload', body: {hardware_supported: true}}]);
  assert.equal(migration.element('hardware-supported').checked, false);

  const current = await fixture(true, true);
  await current.element('reload-session').onclick();
  assert.deepEqual(current.posts, [{url: '/api/launcher/reload', body: {}}]);
  console.log('ARX launcher migration, retained-owner reload and confirmation checks passed');
})().catch(error => {console.error(error); process.exitCode = 1;});
