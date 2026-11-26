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
assert.match(note.textContent, /取值 > 0/);
assert.match(note.textContent, /0.1.*1\/10/);
assert.match(note.textContent, /越小越慢/);
assert.match(note.textContent, /smaller is slower/);
assert.equal(input.min, '0');
assert.equal(input.step, 'any');
context.state.capabilities.embodiment = 'franka'; render();
assert.match(note.textContent, /0 < s ≤ 1/);
assert.equal(input.max, '1');
console.log('Concise bilingual speed guidance and input ranges passed');
