// Exercise the actual recommendation renderer; no browser or live Runtime.
const fs = require('node:fs');
const vm = require('node:vm');
const assert = require('node:assert/strict');
const path = require('node:path');
const source = fs.readFileSync(path.join(__dirname, '../robol2l_deploy/web/experiment.js'), 'utf8');
const renderer = source.slice(source.indexOf('  function renderSpeedRecommendation()'),
  source.indexOf('  function renderMachineStatus()'));
const note = {}, label = {}, input = {value: '0.1', closest: () => ({}), removeAttribute() {}};
const context = vm.createContext({state: {capabilities: {embodiment: 'arx_x5_2023_dual',
  motion_speed: {default_speed_factor: 1}}}, experiment: {status: 'idle'},
  $: id => id === 'exp-speed' ? input : label,
  document: {querySelector: () => note}});
vm.runInContext(renderer, context);
const render = () => vm.runInContext('renderSpeedRecommendation()', context);
render();
assert.match(note.textContent, /输入 0.1 × 默认倍率 1 = 模型收到的推荐倍率 0.1/);
assert.match(note.textContent, /越小越慢/);
assert.match(note.textContent, /对模型仅为建议/);
assert.match(note.textContent, /实际复位倍率为 0.1/);
assert.match(note.textContent, /Smaller is slower/);
context.state.capabilities.motion_speed.default_speed_factor = .4;
input.value = '.25'; render();
assert.match(note.textContent, /输入 0.25 × 默认倍率 0.4 = 模型收到的推荐倍率 0.1/);
context.experiment = {status: 'executing', speed_factor: .25, speed_default_factor: .4,
  speed_recommendation_factor: .1};
context.state.capabilities.motion_speed.default_speed_factor = 2;
render();
assert.match(note.textContent, /默认倍率 0.4/);
assert.match(note.textContent, /实际复位倍率为 0.1/);
context.experiment = {status: 'waiting_reset', speed_factor: .15}; render();
assert.match(note.textContent, /旧记录的推荐倍率为 0.15/);
assert.match(note.textContent, /实际复位倍率为 0.3/);
context.experiment = {status: 'idle'}; input.value = ''; render();
assert.match(note.textContent, /输入正数/);
assert.doesNotMatch(note.textContent, /NaN|undefined/);
console.log('Experiment speed multiplication, frozen baseline, legacy and bilingual rendering passed');
