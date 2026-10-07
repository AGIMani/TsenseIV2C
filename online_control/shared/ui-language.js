import {messages} from './ui-translations.js';

const STORAGE_KEY = 'hand-ui-language';
const normalize = value => value.replace(/\s+/g, ' ').trim();
const phrases = Object.keys(messages).sort((a, b) => b.length - a.length);
const escape = value => value.replace(/[.*+?^${}()|[\]\\]/g, '\\$&');
const phrasePattern = new RegExp(phrases.map(escape).join('|'), 'g');
let language = 'zh-CN';
try { if (localStorage.getItem(STORAGE_KEY) === 'en') language = 'en'; } catch { /* Private browsing. */ }

export function translate(text, locale = language) {
  if (locale !== 'en' || !/[\u3400-\u9fff]/u.test(text)) return text;
  const key = normalize(text);
  if (messages[key]) return text.replace(text.trim(), messages[key]);
  // Dynamic telemetry composes known labels with numbers, units and model IDs.
  const dynamicText = text.replace(/ · 收 (\d+) \/ 发 (\d+) 帧/g, " · RX $1 / TX $2 frames");
  const translated = dynamicText.replace(phrasePattern, phrase => messages[phrase]);
  // Preserve unknown server/device diagnostics verbatim rather than mixing languages.
  return /[\u3400-\u9fff]/u.test(translated) ? text : translated;
}

const textStates = new WeakMap();
const attributeStates = new WeakMap();
const attributes = ['title', 'aria-label', 'placeholder', 'alt'];
const excluded = 'script, style, code, pre, textarea, [translate="no"], [contenteditable="true"]';
function displayText(node) {
  if (!node.parentElement || node.parentElement.closest(excluded)) return;
  const previous = textStates.get(node);
  const source = previous && node.data === previous.output ? previous.source : node.data;
  const output = translate(source);
  if (node.data !== output) {
    // An option without an explicit value derives its value from its label.
    const option = node.parentElement;
    if (option.tagName === 'OPTION' && !option.hasAttribute('value')) option.value = option.text;
    node.data = output;
  }
  textStates.set(node, {source, output});
}
function displayAttributes(element) {
  if (element.closest(excluded)) return;
  let state = attributeStates.get(element);
  if (!state) { state = new Map(); attributeStates.set(element, state); }
  for (const name of attributes) {
    if (!element.hasAttribute(name)) { state.delete(name); continue; }
    const current = element.getAttribute(name), previous = state.get(name);
    const source = previous && current === previous.output ? previous.source : current;
    const output = translate(source);
    if (current !== output) element.setAttribute(name, output);
    state.set(name, {source, output});
  }
}
function renderTree(root) {
  if (root.nodeType === Node.TEXT_NODE) { displayText(root); return; }
  if (root.nodeType !== Node.ELEMENT_NODE || root.closest(excluded)) return;
  displayAttributes(root);
  const walker = document.createTreeWalker(root, NodeFilter.SHOW_ELEMENT | NodeFilter.SHOW_TEXT, {
    acceptNode(node) { return node.nodeType === Node.ELEMENT_NODE && node.matches(excluded) ? NodeFilter.FILTER_REJECT : NodeFilter.FILTER_ACCEPT; },
  });
  while (walker.nextNode()) {
    if (walker.currentNode.nodeType === Node.TEXT_NODE) displayText(walker.currentNode);
    else displayAttributes(walker.currentNode);
  }
}

const button = document.getElementById('language-toggle');
function renderLanguage() {
  document.documentElement.lang = language;
  renderTree(document.head.querySelector('title'));
  renderTree(document.body);
  if (button) {
    button.textContent = language === 'en' ? '中文' : 'EN';
    button.setAttribute('aria-label', language === 'en' ? '切换到中文' : 'Switch to English');
    button.title = language === 'en' ? '切换到中文' : 'Switch to English';
  }
}
export function setLanguage(locale) {
  language = locale === 'en' ? 'en' : 'zh-CN';
  try { localStorage.setItem(STORAGE_KEY, language); } catch { /* Still switch this page. */ }
  renderLanguage();
}
if (button) button.addEventListener('click', () => setLanguage(language === 'en' ? 'zh-CN' : 'en'));
renderLanguage();
// Observe only display changes; never touch input values, IDs or control state.
const observer = new MutationObserver(records => {
  const roots = new Set();
  for (const record of records) {
    if (record.type === 'childList') record.addedNodes.forEach(node => roots.add(node));
    else roots.add(record.target);
  }
  for (const root of roots) if (root.isConnected) renderTree(root);
});
observer.observe(document.body, {subtree: true, childList: true, characterData: true, attributes: true, attributeFilter: attributes});
window.addEventListener('storage', event => {
  if (event.key === STORAGE_KEY) { language = event.newValue === 'en' ? 'en' : 'zh-CN'; renderLanguage(); }
});
