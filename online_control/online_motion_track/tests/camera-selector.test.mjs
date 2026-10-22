import assert from 'node:assert/strict';
import test from 'node:test';
import {bindCameraSelector} from '../static/camera-selector.js';

const camera = (deviceId, label = deviceId) => ({kind: 'videoinput', deviceId, label});
const settle = () => new Promise(resolve => setImmediate(resolve));
function fixture() {
  const select = new EventTarget();
  Object.assign(select, {
    value: '', disabled: false,
    options: [{value: '', textContent: '默认摄像头'}],
    ownerDocument: {createElement: () => ({})},
    replaceChildren(...options) { this.options = options; this.value = options[0].value; },
    setAttribute() {}, removeAttribute() {},
  });
  const mediaDevices = new EventTarget();
  const state = {devices: [camera('internal')], scans: 0, permissions: 0, stops: 0, errors: []};
  mediaDevices.enumerateDevices = async () => { state.scans++; return state.devices; };
  mediaDevices.getUserMedia = async constraints => {
    state.permissions++;
    assert.deepEqual(constraints, {audio: false, video: true});
    state.devices = [camera('internal'), camera('usb')];
    return {getTracks: () => [{stop() { state.stops++; }}]};
  };
  const picker = bindCameraSelector(select, {mediaDevices, onError: error => state.errors.push(error)});
  function open(type = 'pointerdown', properties = {button: 0}) {
    const event = new Event(type);
    Object.assign(event, properties);
    select.dispatchEvent(event);
  }
  return {select, mediaDevices, state, picker, open};
}

test('each opening refreshes hotplugged cameras and retains the selected device', async () => {
  const {select, state, open} = fixture();
  open(); await settle();
  select.value = 'internal';
  state.devices.push(camera('usb'), {kind: 'audioinput', deviceId: 'microphone'});
  open(); await settle();
  assert.equal(state.scans, 2);
  assert.deepEqual(select.options.map(option => option.value), ['', 'internal', 'usb']);
  assert.equal(select.value, 'internal');
  state.devices = [camera('usb')];
  open(); await settle();
  assert.equal(state.scans, 3);
  assert.deepEqual(select.options.map(option => option.value), ['', 'usb']);
  assert.equal(select.value, '');
  assert.equal(state.permissions, 0);
});

test('only user interaction requests permission and releases the temporary camera', async () => {
  const {select, mediaDevices, state, picker, open} = fixture();
  state.devices = [camera('', '')];
  await picker.refresh();
  mediaDevices.dispatchEvent(new Event('devicechange')); await settle();
  assert.equal(state.permissions, 0);
  open(); await settle();
  assert.equal(state.permissions, 1);
  assert.equal(state.stops, 1);
  assert.deepEqual(select.options.map(option => option.value), ['', 'internal', 'usb']);
  assert.deepEqual(state.errors, []);
});

test('permission is released even if enumeration fails after opening the camera', async () => {
  const {select, mediaDevices, state, open} = fixture();
  const failure = new Error('enumeration failed');
  mediaDevices.enumerateDevices = async () => {
    if (state.permissions) throw failure;
    return [camera('', '')];
  };
  open(); await settle();
  assert.equal(state.stops, 1);
  assert.deepEqual(state.errors, [failure]);
  assert.deepEqual(select.options.map(option => option.value), ['']);
});

test('denied permission preserves the list and another click can retry', async () => {
  const {select, mediaDevices, state, open} = fixture();
  state.devices = [camera('masked', '')];
  const acquire = mediaDevices.getUserMedia;
  mediaDevices.getUserMedia = async () => { throw new DOMException('denied', 'NotAllowedError'); };
  open(); await settle();
  assert.equal(state.errors[0].name, 'NotAllowedError');
  assert.deepEqual(select.options.map(option => option.value), ['']);
  mediaDevices.getUserMedia = acquire;
  open(); await settle();
  assert.deepEqual(select.options.map(option => option.value), ['', 'internal', 'usb']);
  assert.equal(state.stops, 1);
});

test('a click during the initial scan upgrades permission without duplicate prompts', async () => {
  const {mediaDevices, state, picker, open} = fixture();
  let resolveScan;
  mediaDevices.enumerateDevices = () => state.permissions
    ? Promise.resolve(state.devices)
    : new Promise(resolve => { resolveScan = resolve; });
  const pending = picker.refresh();
  open(); open();
  resolveScan([camera('', '')]);
  await pending;
  assert.equal(state.permissions, 1);
  assert.equal(state.stops, 1);
});

test('selection changes during an asynchronous scan are not overwritten', async () => {
  const {select, mediaDevices, picker} = fixture();
  let resolveScan;
  mediaDevices.enumerateDevices = () => new Promise(resolve => { resolveScan = resolve; });
  const pending = picker.refresh();
  select.value = 'usb';
  resolveScan([camera('internal'), camera('usb')]);
  await pending;
  assert.equal(select.value, 'usb');
});

test('keyboard opening scans cameras, while disabled controls do not request access', async () => {
  const {select, state, open} = fixture();
  open('keydown', {key: 'ArrowDown', repeat: false}); await settle();
  assert.equal(state.scans, 1);
  select.disabled = true;
  open(); open('keydown', {key: 'Enter', repeat: false}); await settle();
  assert.equal(state.scans, 1);
  assert.equal(state.permissions, 0);
});

test('unplugging every camera removes stale choices even when permission probing finds none', async () => {
  const {select, mediaDevices, state, open} = fixture();
  open(); await settle();
  select.value = 'internal';
  state.devices = [];
  mediaDevices.getUserMedia = async () => { throw new DOMException('no cameras', 'NotFoundError'); };
  open(); await settle();
  assert.deepEqual(select.options.map(option => option.value), ['']);
  assert.equal(select.value, '');
  assert.equal(state.errors[0].name, 'NotFoundError');
});
