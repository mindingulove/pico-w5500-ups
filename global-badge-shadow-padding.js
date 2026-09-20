const HEADER_TAG = "hui-view-header";
const TOP_PADDING = "4px";
const BOTTOM_PADDING = "12px";
const ROOT_HOSTS = new Set([
  "home-assistant",
  "home-assistant-main",
  "ha-panel-lovelace",
  "hui-root",
]);
const observedRoots = new WeakSet();
const observedHeaders = new WeakSet();

window.__globalBadgeShadowPadding = {
  version: 7,
  discoveredHeaders: 0,
  patchedHeaders: 0,
  observedRoots: 0,
};

function isLovelaceRouteHost(element) {
  const tag = element?.localName;
  return (
    ROOT_HOSTS.has(tag) ||
    (tag?.startsWith("hui-") && tag.endsWith("-view"))
  );
}

function applyHeaderPadding(header) {
  const badgeScroller = header.shadowRoot?.querySelector(".badges.scroll");
  if (!badgeScroller) return;

  const topPaddingChanged =
    badgeScroller.style.getPropertyValue("padding-top") !== TOP_PADDING ||
    badgeScroller.style.getPropertyPriority("padding-top") !== "important";
  const bottomPaddingChanged =
    badgeScroller.style.getPropertyValue("padding-bottom") !== BOTTOM_PADDING ||
    badgeScroller.style.getPropertyPriority("padding-bottom") !== "important";
  const sizingChanged =
    badgeScroller.style.getPropertyValue("box-sizing") !== "content-box" ||
    badgeScroller.style.getPropertyPriority("box-sizing") !== "important";

  if (topPaddingChanged) {
    badgeScroller.style.setProperty("padding-top", TOP_PADDING, "important");
  }
  if (bottomPaddingChanged) {
    badgeScroller.style.setProperty(
      "padding-bottom",
      BOTTOM_PADDING,
      "important",
    );
  }
  if (sizingChanged) {
    badgeScroller.style.setProperty("box-sizing", "content-box", "important");
  }

  header.dataset.globalBadgeShadowPadding = "v7";
  if (topPaddingChanged || bottomPaddingChanged || sizingChanged) {
    window.__globalBadgeShadowPadding.patchedHeaders += 1;
  }
}

function observeHeader(header) {
  applyHeaderPadding(header);
  if (!header.shadowRoot || observedHeaders.has(header)) return;

  observedHeaders.add(header);
  window.__globalBadgeShadowPadding.discoveredHeaders += 1;

  const observer = new MutationObserver(() => {
    queueMicrotask(() => applyHeaderPadding(header));
  });
  observer.observe(header.shadowRoot, { childList: true, subtree: true });
}

function inspectElement(element) {
  if (element.localName === HEADER_TAG) {
    observeHeader(element);
  }
  if (isLovelaceRouteHost(element) && element.shadowRoot) {
    observeRoot(element.shadowRoot);
  }
}

function scanTree(root) {
  if (root instanceof Element) inspectElement(root);
  if (!root.querySelectorAll) return;

  for (const element of root.querySelectorAll("*")) {
    if (element.localName === HEADER_TAG || isLovelaceRouteHost(element)) {
      inspectElement(element);
    }
  }
}

function observeRoot(root) {
  if (!root || observedRoots.has(root)) return;

  observedRoots.add(root);
  window.__globalBadgeShadowPadding.observedRoots += 1;

  const observer = new MutationObserver((mutations) => {
    for (const mutation of mutations) {
      for (const node of mutation.addedNodes) {
        if (node instanceof Element || node instanceof DocumentFragment) {
          scanTree(node);
        }
      }
    }
  });
  observer.observe(root, { childList: true, subtree: true });
  scanTree(root);
}

const originalAttachShadow = Element.prototype.attachShadow;
if (!Element.prototype.__globalBadgeShadowPaddingV7) {
  Element.prototype.attachShadow = function attachShadow(init) {
    const shadowRoot = originalAttachShadow.call(this, init);
    if (this.localName === HEADER_TAG || isLovelaceRouteHost(this)) {
      queueMicrotask(() => observeRoot(shadowRoot));
    }
    return shadowRoot;
  };
  Object.defineProperty(Element.prototype, "__globalBadgeShadowPaddingV7", {
    value: true,
  });
}

observeRoot(document);
