// rqt_tf_tree-style directed frame graph for the System view.
//
// This module is intentionally dependency-free and side-effect-free at import
// time so both the browser and Node behaviour tests can drive it. Layout is
// pure data; rendering only ever writes frame names through textContent, never
// innerHTML, so a hostile frame id can never become markup.
//
// Design notes:
// * The backend already returns a forest (multiple roots, plus a detached edge
//   list for malformed/cycle/extra-parent edges). This module only lays that
//   payload out; it never invents publisher, rate or transform numbers.
// * All traversals are iterative so a 128-deep (or deeper) chain can never blow
//   the JS call stack.
// * Node widths are capped, so a very long frame id truncates in the graph and
//   keeps its full name in a <title> / aria-label instead of stretching the
//   canvas without bound.

const SVG_NS = "http://www.w3.org/2000/svg";

export const TF_GRAPH_MAX_LABEL = 22;
export const TF_GRAPH_NODE_HEIGHT = 30;
// Slot pitch must clear the widest possible node (estimateNodeWidth caps at
// 196) plus a visible gutter so long-name siblings never overlap.
export const TF_GRAPH_COL_WIDTH = 224;
export const TF_GRAPH_ROW_HEIGHT = 86;
export const TF_GRAPH_PAD = 26;

export function formatTfAge(age) {
  if (!Number.isFinite(age)) return "--";
  if (age < 1) return `${Math.round(age * 1000)} ms`;
  return `${age.toFixed(1)} s`;
}

export function truncateFrameLabel(frame, max = TF_GRAPH_MAX_LABEL) {
  const text = String(frame == null ? "" : frame);
  if (max < 2 || text.length <= max) return text;
  return `${text.slice(0, max - 1)}…`;
}

export function tfEdgeKind(edge) {
  return edge && edge.static ? "static" : "dynamic";
}

export function estimateNodeWidth(label) {
  const text = String(label == null ? "" : label);
  return Math.min(196, Math.max(58, text.length * 7.4 + 24));
}

// --- Model -----------------------------------------------------------------

export function buildTfGraphModel(data) {
  const source = data && typeof data === "object" ? data : {};
  const trees = Array.isArray(source.trees) ? source.trees : [];
  const detachedIn = Array.isArray(source.detached) ? source.detached : [];
  const nodes = new Map();
  const edges = [];
  const roots = [];
  const stack = [];
  for (let i = trees.length - 1; i >= 0; i -= 1) {
    const tree = trees[i];
    if (tree) stack.push({ node: tree, parent: null });
  }

  while (stack.length) {
    const item = stack.pop();
    const raw = item.node || {};
    const frame = String(raw.frame == null ? "" : raw.frame);
    if (!frame || nodes.has(frame)) continue;
    const rawChildren = Array.isArray(raw.children) ? raw.children : [];
    const validChildren = [];
    for (const child of rawChildren) {
      const childFrame = String(child && child.frame != null ? child.frame : "");
      if (childFrame) validChildren.push({ frame: childFrame, node: child });
    }
    const record = {
      id: frame,
      frame,
      parent: item.parent,
      edge: raw.edge || null,
      children: validChildren.map((child) => child.frame),
      detached: false,
    };
    nodes.set(frame, record);
    if (item.parent) {
      edges.push({
        id: `${item.parent}->${frame}`,
        parent: item.parent,
        child: frame,
        edge: record.edge,
        detached: false,
      });
    } else {
      roots.push(frame);
    }
    for (let i = validChildren.length - 1; i >= 0; i -= 1) {
      stack.push({ node: validChildren[i].node, parent: frame });
    }
  }

  // Depth is resolvable in insertion order because a parent is always inserted
  // before its children by the pre-order walk above.
  const depth = new Map();
  for (const record of nodes.values()) {
    if (record.parent && depth.has(record.parent)) {
      depth.set(record.id, depth.get(record.parent) + 1);
    } else {
      depth.set(record.id, 0);
    }
  }

  const detached = detachedIn.map((edge) => ({
    parent: String(edge && edge.parent != null ? edge.parent : ""),
    child: String(edge && edge.child != null ? edge.child : ""),
    edge: edge || {},
    reason: String(edge && edge.reason ? edge.reason : "unplaced"),
  }));

  const frameCount = Number.isFinite(source.frame_count) ? source.frame_count : nodes.size;
  return { nodes, edges, roots, detached, depth, frameCount };
}

export function graphTopologySignature(data) {
  const model = buildTfGraphModel(data);
  const nodes = [...model.nodes.keys()].sort().join(",");
  const edges = model.edges
    .map((edge) => `${edge.parent}>${edge.child}:${edge.edge && edge.edge.static ? 1 : 0}`)
    .sort()
    .join(",");
  return `${nodes}|${edges}`;
}

// --- Layout ----------------------------------------------------------------

export function layoutTfGraph(data, options = {}) {
  const model = buildTfGraphModel(data);
  const colWidth = Number.isFinite(options.colWidth) ? options.colWidth : TF_GRAPH_COL_WIDTH;
  const rowHeight = Number.isFinite(options.rowHeight) ? options.rowHeight : TF_GRAPH_ROW_HEIGHT;
  const pad = Number.isFinite(options.pad) ? options.pad : TF_GRAPH_PAD;

  const childrenOf = new Map();
  for (const record of model.nodes.values()) {
    childrenOf.set(
      record.id,
      record.children.filter((child) => model.nodes.has(child)),
    );
  }

  // Classic tidy-tree x assignment: leaves take successive slots, parents sit
  // over the mean of their children. Fully iterative.
  const slot = new Map();
  let nextLeaf = 0;
  const stack = model.roots.map((id) => ({ id, phase: 0 }));
  while (stack.length) {
    const item = stack.pop();
    const kids = childrenOf.get(item.id) || [];
    if (item.phase === 0) {
      stack.push({ id: item.id, phase: 1 });
      for (let i = kids.length - 1; i >= 0; i -= 1) stack.push({ id: kids[i], phase: 0 });
    } else if (!kids.length) {
      slot.set(item.id, nextLeaf);
      nextLeaf += 1;
    } else {
      let sum = 0;
      let count = 0;
      for (const kid of kids) {
        if (slot.has(kid)) {
          sum += slot.get(kid);
          count += 1;
        }
      }
      slot.set(item.id, count ? sum / count : nextLeaf++);
    }
  }

  const labelLimit = Number.isFinite(options.maxLabel) ? options.maxLabel : TF_GRAPH_MAX_LABEL;
  const nodeHeight = Number.isFinite(options.nodeHeight) ? options.nodeHeight : TF_GRAPH_NODE_HEIGHT;
  const nodes = [];
  const nodeById = new Map();
  let maxDepth = 0;
  let maxRight = 0;
  for (const record of model.nodes.values()) {
    const depth = model.depth.get(record.id) || 0;
    const label = truncateFrameLabel(record.frame, labelLimit);
    const width = estimateNodeWidth(label);
    const slotIndex = slot.has(record.id) ? slot.get(record.id) : nextLeaf++;
    const x = pad + slotIndex * colWidth + colWidth / 2;
    const y = pad + depth * rowHeight;
    const node = {
      ...record,
      label,
      fullLabel: record.frame,
      width,
      height: nodeHeight,
      depth,
      x,
      y,
    };
    nodes.push(node);
    nodeById.set(node.id, node);
    if (depth > maxDepth) maxDepth = depth;
    if (x + width / 2 > maxRight) maxRight = x + width / 2;
  }

  const edges = [];
  for (const edge of model.edges) {
    const parent = nodeById.get(edge.parent);
    const child = nodeById.get(edge.child);
    if (!parent || !child) continue;
    edges.push({
      ...edge,
      kind: tfEdgeKind(edge.edge),
      stale: Boolean(edge.edge && edge.edge.stale),
      x1: parent.x,
      y1: parent.y + parent.height,
      x2: child.x,
      y2: child.y,
    });
  }

  const detached = model.detached.map((edge) => ({
    ...edge,
    parentNode: nodeById.get(edge.parent) || null,
    childNode: nodeById.get(edge.child) || null,
  }));

  const graphBottom = pad + (maxDepth + 1) * rowHeight;
  const detachedTop = detached.length ? graphBottom + 14 : graphBottom;
  const width = Math.max(maxRight + pad, options.minWidth || 320);
  const height = Math.max(
    detachedTop + detached.length * 26 + pad,
    options.minHeight || 220,
  );

  return {
    model,
    nodes,
    nodeById,
    edges,
    detached,
    roots: model.roots,
    frameCount: model.frameCount,
    maxDepth,
    width,
    height,
    emptyMessage: String(options.emptyMessage || (data && data.message) || "waiting for TF"),
  };
}

// --- SVG rendering ---------------------------------------------------------

function svgEl(doc, tag) {
  return doc.createElementNS(SVG_NS, tag);
}

function setText(doc, element, text) {
  element.textContent = String(text == null ? "" : text);
  return element;
}

function markerFor(kind, stale) {
  if (kind === "static") return "tfArrowStatic";
  if (stale) return "tfArrowStale";
  return "tfArrowDynamic";
}

export function buildTfGraphSvg(doc, layout, options = {}) {
  const width = Number.isFinite(options.width) && options.width > 0 ? options.width : layout.width;
  const height =
    Number.isFinite(options.height) && options.height > 0 ? options.height : layout.height;
  const svg = svgEl(doc, "svg");
  svg.setAttribute("class", "tf-graph-svg");
  svg.setAttribute("viewBox", `0 0 ${width} ${height}`);
  svg.setAttribute("preserveAspectRatio", "xMidYMid meet");
  svg.setAttribute("role", "img");
  svg.setAttribute("aria-label", `${layout.nodes.length} TF frames`);

  const defs = svgEl(doc, "defs");
  for (const [id, color] of [
    ["tfArrowDynamic", "#38bdf8"],
    ["tfArrowStatic", "#34d399"],
    ["tfArrowStale", "#f87171"],
  ]) {
    const marker = svgEl(doc, "marker");
    marker.setAttribute("id", id);
    marker.setAttribute("viewBox", "0 0 10 10");
    marker.setAttribute("refX", "9");
    marker.setAttribute("refY", "5");
    marker.setAttribute("markerWidth", "7");
    marker.setAttribute("markerHeight", "7");
    marker.setAttribute("orient", "auto-start-reverse");
    const path = svgEl(doc, "path");
    path.setAttribute("d", "M 0 0 L 10 5 L 0 10 z");
    path.setAttribute("fill", color);
    marker.appendChild(path);
    defs.appendChild(marker);
  }
  svg.appendChild(defs);

  const viewport = svgEl(doc, "g");
  viewport.setAttribute("class", "tf-graph-viewport");
  svg.appendChild(viewport);

  const edgeLayer = svgEl(doc, "g");
  edgeLayer.setAttribute("class", "tf-graph-edges");
  const detachedEdgeLayer = svgEl(doc, "g");
  detachedEdgeLayer.setAttribute("class", "tf-graph-detached-edges");
  const nodeLayer = svgEl(doc, "g");
  nodeLayer.setAttribute("class", "tf-graph-nodes");
  const detachedLayer = svgEl(doc, "g");
  detachedLayer.setAttribute("class", "tf-graph-detached");
  viewport.appendChild(edgeLayer);
  viewport.appendChild(detachedEdgeLayer);
  viewport.appendChild(detachedLayer);
  viewport.appendChild(nodeLayer);

  if (!layout.nodes.length && !layout.detached.length) {
    const placeholder = svgEl(doc, "text");
    placeholder.setAttribute("class", "tf-graph-placeholder");
    placeholder.setAttribute("x", String(width / 2));
    placeholder.setAttribute("y", String(Math.min(height / 2, 80)));
    placeholder.setAttribute("text-anchor", "middle");
    setText(doc, placeholder, layout.emptyMessage);
    viewport.appendChild(placeholder);
    return { svg, viewport, edgeLayer, nodeLayer, detachedLayer, nodes: [], edges: [] };
  }

  const edgeRecords = [];
  for (const edge of layout.edges) {
    const curve = svgEl(doc, "path");
    const bend = Math.max(18, (edge.y2 - edge.y1) / 2);
    curve.setAttribute(
      "d",
      `M ${edge.x1} ${edge.y1} C ${edge.x1} ${edge.y1 + bend}, ${edge.x2} ${edge.y2 - bend}, ${edge.x2} ${edge.y2}`,
    );
    curve.setAttribute("class", `tf-edge tf-edge-${edge.kind}${edge.stale ? " is-stale" : ""}`);
    curve.setAttribute("marker-end", `url(#${markerFor(edge.kind, edge.stale)})`);
    setText(doc, curve, `${edge.parent} to ${edge.child}`);
    edgeLayer.appendChild(curve);
    edgeRecords.push({ ...edge, element: curve });
  }

  const detachedRecords = [];
  for (const edge of layout.detached) {
    if (edge.parentNode && edge.childNode) {
      const curve = svgEl(doc, "path");
      const bend = Math.max(24, Math.abs(edge.childNode.x - edge.parentNode.x) / 2 + 20);
      curve.setAttribute(
        "d",
        `M ${edge.parentNode.x} ${edge.parentNode.y + edge.parentNode.height} C ${edge.parentNode.x} ${edge.parentNode.y + bend}, ${edge.childNode.x} ${edge.childNode.y - bend}, ${edge.childNode.x} ${edge.childNode.y}`,
      );
      curve.setAttribute("class", "tf-edge tf-edge-detached");
      curve.setAttribute("marker-end", "url(#tfArrowStale)");
      detachedEdgeLayer.appendChild(curve);
    }
    const item = svgEl(doc, "g");
    item.setAttribute("class", "tf-detached-item");
    const text = svgEl(doc, "text");
    text.setAttribute("x", String(TF_GRAPH_PAD));
    text.setAttribute("y", String(layout.height - 10 - (layout.detached.length - 1 - detachedRecords.length) * 26));
    setText(
      doc,
      text,
      `${edge.parent || "?"} → ${edge.child || "?"} (${tfEdgeKind(edge.edge)}): ${edge.reason}`,
    );
    item.appendChild(text);
    detachedLayer.appendChild(item);
    detachedRecords.push({ ...edge, element: item });
  }

  const nodeRecords = [];
  for (const node of layout.nodes) {
    const group = svgEl(doc, "g");
    group.setAttribute("class", `tf-node${node.edge ? ` tf-node-${tfEdgeKind(node.edge)}` : " tf-node-root"}${node.edge && node.edge.stale ? " is-stale" : ""}`);
    group.setAttribute("data-frame", node.frame);
    group.setAttribute("tabindex", "0");
    group.setAttribute("role", "button");
    group.setAttribute("aria-label", node.edge ? `${node.parent} to ${node.frame}` : `root frame ${node.frame}`);

    const rect = svgEl(doc, "rect");
    rect.setAttribute("x", String(node.x - node.width / 2));
    rect.setAttribute("y", String(node.y));
    rect.setAttribute("width", String(node.width));
    rect.setAttribute("height", String(node.height));
    rect.setAttribute("rx", "8");
    group.appendChild(rect);

    const label = svgEl(doc, "text");
    label.setAttribute("class", "tf-frame-text");
    label.setAttribute("x", String(node.x));
    label.setAttribute("y", String(node.y + node.height / 2));
    label.setAttribute("text-anchor", "middle");
    label.setAttribute("dominant-baseline", "central");
    setText(doc, label, node.label);
    group.appendChild(label);

    const title = svgEl(doc, "title");
    setText(doc, title, node.edge ? `${node.edge.parent} → ${node.frame}` : node.frame);
    group.appendChild(title);

    nodeLayer.appendChild(group);
    nodeRecords.push({ id: node.id, node, element: group });
  }

  return {
    svg,
    viewport,
    edgeLayer,
    nodeLayer,
    detachedLayer,
    nodes: nodeRecords,
    edges: edgeRecords,
    detached: detachedRecords,
  };
}

// --- Interactive view ------------------------------------------------------

function clamp(value, min, max) {
  return Math.min(max, Math.max(min, value));
}

export class TfGraphView {
  constructor(host, options = {}) {
    this.host = host || null;
    this.doc =
      options.document || (typeof document !== "undefined" ? document : null);
    this.options = options;
    this.onSelect = typeof options.onSelect === "function" ? options.onSelect : null;
    this.active = options.active !== false;
    this.transform = { x: 0, y: 0, k: 1 };
    this.layout = null;
    this.scene = null;
    this.selectedId = null;
    this.topology = "";
    this.initialized = false;
    this.needsFit = true;
    this.lastWidth = 0;
    this.lastHeight = 0;
    this.pointers = new Map();
    // Host listeners live for the view's whole lifetime; scene listeners are
    // torn down before every rebuild so replaced SVG nodes are never retained.
    this.hostListeners = [];
    this.sceneListeners = [];
    this.minZoom = 0.12;
    this.maxZoom = 4;
    this._gesture = null;
    this._suppressClick = false;
    this._capturedId = null;
    this._pinchDistance = 0;
    this.resizeObserver = null;
    this._bindHost();
    if (
      this.host &&
      typeof ResizeObserver !== "undefined" &&
      typeof ResizeObserver === "function"
    ) {
      try {
        this.resizeObserver = new ResizeObserver(() => this.handleResize());
        this.resizeObserver.observe(this.host);
      } catch (_error) {
        this.resizeObserver = null;
      }
    }
  }

  _addListener(bucket, target, type, handler, options) {
    if (!target || typeof target.addEventListener !== "function") return;
    target.addEventListener(type, handler, options);
    bucket.push({ target, type, handler, options });
  }

  _listenHost(target, type, handler, options) {
    this._addListener(this.hostListeners, target, type, handler, options);
  }

  _listenScene(target, type, handler, options) {
    this._addListener(this.sceneListeners, target, type, handler, options);
  }

  _releaseSceneListeners() {
    for (const { target, type, handler, options } of this.sceneListeners) {
      if (target && typeof target.removeEventListener === "function") {
        target.removeEventListener(type, handler, options);
      }
    }
    this.sceneListeners = [];
  }

  _bindHost() {
    this._listenHost(this.host, "wheel", (event) => this.handleWheel(event), { passive: false });
    this._listenHost(this.host, "pointerdown", (event) => this.handlePointerDown(event));
    this._listenHost(this.host, "pointermove", (event) => this.handlePointerMove(event));
    this._listenHost(this.host, "pointerup", (event) => this.handlePointerUp(event));
    this._listenHost(this.host, "pointercancel", (event) => this.handlePointerUp(event));
    this._listenHost(this.host, "click", (event) => this.handleClick(event));
    this._listenHost(this.host, "keydown", (event) => this.handleKeyDown(event));
  }

  hostSize() {
    if (!this.host) return { width: 0, height: 0 };
    return {
      width: Number(this.host.clientWidth) || 0,
      height: Number(this.host.clientHeight) || 0,
    };
  }

  setActive(active) {
    const next = Boolean(active);
    if (!next && this.active) {
      // Leaving the view must not leave a stale gesture to resume on return.
      this.pointers.clear();
      this._pinchDistance = 0;
      this._gesture = null;
      this._suppressClick = false;
      this._releasePointerCapture();
    }
    this.active = next;
    if (this.active && this.needsFit) this.fit();
    return this.active;
  }

  update(data) {
    if (!this.doc || !this.host) return null;
    const layout = layoutTfGraph(data, {
      emptyMessage:
        data && data.status === "waiting"
          ? data.message || "waiting for TF"
          : "TF data unavailable.",
    });
    const topology = graphTopologySignature(data);
    const topologyChanged = topology !== this.topology;
    this.layout = layout;
    this.topology = topology;
    let selectionLost = false;
    if (this.selectedId && !layout.nodeById.has(this.selectedId)) {
      this.selectedId = null;
      selectionLost = true;
    }
    this.render();
    // Re-emit the selected node every update so the details panel shows the
    // freshly received parent/age/stale fields, not the values from the click.
    if (this.onSelect && this.selectedId && layout.nodeById.has(this.selectedId)) {
      this.onSelect(layout.nodeById.get(this.selectedId));
    } else if (this.onSelect && selectionLost) {
      this.onSelect(null);
    }
    const hasNodes = layout.nodes.length > 0 || layout.detached.length > 0;
    // A waiting/empty payload must not consume the one-time fit: the first real
    // non-empty graph is what needs to be framed.
    if (!this.initialized && hasNodes) {
      this.initialized = true;
      this.fit();
    } else if (this.needsFit) {
      this.fit();
    } else if (topologyChanged) {
      // True topology change: layout moved, but the user's pan/zoom is kept.
      this._applyTransform();
    }
    return layout;
  }

  render() {
    if (!this.layout || !this.doc || !this.host) return null;
    const active = this.doc.activeElement;
    const focusedFrame =
      active && typeof active.getAttribute === "function"
        ? active.getAttribute("data-frame")
        : null;
    // Drop listeners bound to the previous SVG before it is thrown away.
    this._releaseSceneListeners();
    const size = this.hostSize();
    const scene = buildTfGraphSvg(this.doc, this.layout, {
      width: size.width || this.layout.width,
      height: size.height || this.layout.height,
    });
    this.scene = scene;
    if (typeof this.host.replaceChildren === "function") {
      this.host.replaceChildren(scene.svg);
    } else if (this.host) {
      this.host.textContent = "";
      this.host.appendChild(scene.svg);
    }
    // Selection is resolved from host pointer/click delegation, so per-node
    // listeners are only needed for keyboard activation.
    for (const record of scene.nodes) {
      this._listenScene(record.element, "keydown", (event) => {
        if (!this.active) return;
        if (event.key === "Enter" || event.key === " " || event.key === "Spacebar") {
          if (typeof event.preventDefault === "function") event.preventDefault();
          this.select(record.id);
        }
      });
    }
    if (focusedFrame) {
      const record = scene.nodes.find((entry) => entry.id === focusedFrame);
      if (record && typeof record.element.focus === "function") record.element.focus();
    }
    if (this.selectedId) this._paintSelection();
    this._applyTransform();
    return scene;
  }

  _frameFromEvent(event) {
    const target = event && event.target;
    if (!target) return "";
    if (typeof target.closest === "function") {
      const found = target.closest("[data-frame]");
      if (found && typeof found.getAttribute === "function") {
        return found.getAttribute("data-frame") || "";
      }
    }
    let element = target;
    let guard = 0;
    while (element && guard < 64) {
      if (typeof element.getAttribute === "function") {
        const frame = element.getAttribute("data-frame");
        if (frame) return frame;
      }
      element = element.parentNode || null;
      guard += 1;
    }
    return "";
  }

  _localPoint(event) {
    const rect =
      this.host && typeof this.host.getBoundingClientRect === "function"
        ? this.host.getBoundingClientRect()
        : null;
    const left = rect ? Number(rect.left) || 0 : 0;
    const top = rect ? Number(rect.top) || 0 : 0;
    const hasClient =
      Number.isFinite(event && event.clientX) && Number.isFinite(event && event.clientY);
    if (!hasClient) return { x: undefined, y: undefined };
    return { x: event.clientX - left, y: event.clientY - top };
  }

  _capturePointer(id) {
    if (!this.host || typeof this.host.setPointerCapture !== "function") return;
    try {
      this.host.setPointerCapture(id);
      this._capturedId = id;
    } catch (_error) {
      // Pointer capture is an enhancement.
    }
  }

  _releasePointerCapture() {
    const id = this._capturedId;
    this._capturedId = null;
    if (id === null || !this.host) return;
    if (typeof this.host.releasePointerCapture === "function") {
      try {
        this.host.releasePointerCapture(id);
      } catch (_error) {
        // Pointer capture is an enhancement.
      }
    }
  }

  _updateViewBox() {
    if (!this.scene || !this.layout) return;
    const size = this.hostSize();
    const width = size.width || this.layout.width;
    const height = size.height || this.layout.height;
    this.scene.svg.setAttribute("viewBox", `0 0 ${width} ${height}`);
  }

  _applyTransform() {
    if (!this.scene || !this.scene.viewport) return;
    const { x, y, k } = this.transform;
    this.scene.viewport.setAttribute("transform", `translate(${x} ${y}) scale(${k})`);
  }

  fit() {
    if (!this.layout || !this.host) return false;
    const size = this.hostSize();
    if (!size.width || !size.height) {
      this.needsFit = true;
      return false;
    }
    this.needsFit = false;
    const pad = 18;
    const availableWidth = Math.max(1, size.width - pad * 2);
    const availableHeight = Math.max(1, size.height - pad * 2);
    const fitted = Math.min(
      availableWidth / Math.max(1, this.layout.width),
      availableHeight / Math.max(1, this.layout.height),
      1.25,
    );
    // Deep/wide graphs need a fit below the old 0.12 floor; otherwise the whole
    // graph still overflows and zoom-in would snap straight back up to 0.12.
    const k = Math.max(Number.EPSILON, fitted);
    this.minZoom = Math.min(0.12, k);
    this.transform = {
      k,
      x: Math.max(pad, (size.width - this.layout.width * k) / 2),
      y: Math.max(pad, (size.height - this.layout.height * k) / 2),
    };
    this._updateViewBox();
    this._applyTransform();
    return true;
  }

  reset() {
    return this.fit();
  }

  zoomBy(factor, centerX, centerY) {
    if (!this.layout) return this.transform.k;
    const size = this.hostSize();
    const previous = this.transform.k;
    const next = clamp(previous * factor, this.minZoom, this.maxZoom);
    if (next === previous) return previous;
    const cx = Number.isFinite(centerX) ? centerX : size.width / 2;
    const cy = Number.isFinite(centerY) ? centerY : size.height / 2;
    this.transform.x = cx - (cx - this.transform.x) * (next / previous);
    this.transform.y = cy - (cy - this.transform.y) * (next / previous);
    this.transform.k = next;
    this.needsFit = false;
    this._applyTransform();
    return next;
  }

  panBy(dx, dy) {
    this.transform.x += Number(dx) || 0;
    this.transform.y += Number(dy) || 0;
    this.needsFit = false;
    this._applyTransform();
  }

  select(id) {
    if (!id) return false;
    this.selectedId = id;
    this._paintSelection();
    if (this.onSelect && this.layout) {
      const node = this.layout.nodeById.get(id) || null;
      this.onSelect(node);
    }
    return true;
  }

  _paintSelection() {
    if (!this.scene) return;
    for (const record of this.scene.nodes) {
      const selected = record.id === this.selectedId;
      const current = record.element.getAttribute("class") || "";
      if (selected && !current.includes("is-selected")) {
        record.element.setAttribute("class", `${current} is-selected`);
      } else if (!selected && current.includes("is-selected")) {
        record.element.setAttribute("class", current.replace(/\s*is-selected/, ""));
      }
    }
  }

  handleWheel(event) {
    if (!this.active) return;
    if (event && typeof event.preventDefault === "function") event.preventDefault();
    const delta = Number(event && event.deltaY) || 0;
    const factor = Math.exp(-delta * 0.0015);
    // Anchor at the host-local pointer, independent of which SVG child was the
    // event target (offsetX/Y is relative to that child, not the host).
    const point = this._localPoint(event);
    this.zoomBy(factor, point.x, point.y);
  }

  handlePointerDown(event) {
    if (!this.active) return;
    const id = event && event.pointerId;
    if (id === undefined || id === null) return;
    const button = event.button === undefined || event.button === null ? 0 : Number(event.button);
    // Only the primary button pans/selects; right-click must not start a pan.
    if (button !== 0) return;
    this._suppressClick = false;
    const x = Number(event.clientX) || 0;
    const y = Number(event.clientY) || 0;
    this.pointers.set(id, { x, y });
    if (!this._gesture) {
      this._gesture = {
        id,
        startX: x,
        startY: y,
        frame: this._frameFromEvent(event),
        moved: false,
      };
    } else if (this._gesture.id !== id) {
      // A second contact makes this a pinch, never a frame tap.
      this._gesture.moved = true;
      this._suppressClick = true;
    }
  }

  handlePointerMove(event) {
    if (!this.active) return;
    const id = event && event.pointerId;
    if (id === undefined || id === null || !this.pointers.has(id)) return;
    const previous = this.pointers.get(id);
    const next = { x: Number(event.clientX) || 0, y: Number(event.clientY) || 0 };
    this.pointers.set(id, next);
    if (this.pointers.size === 1) {
      const gesture = this._gesture && this._gesture.id === id ? this._gesture : null;
      if (gesture) {
        const travelled = Math.hypot(next.x - gesture.startX, next.y - gesture.startY);
        if (!gesture.moved && travelled > 4) {
          gesture.moved = true;
          this._capturePointer(id);
        }
      }
      if (!gesture || gesture.moved) {
        this.panBy(next.x - previous.x, next.y - previous.y);
      }
      if (event && typeof event.preventDefault === "function") event.preventDefault();
    } else if (this.pointers.size === 2) {
      const values = [...this.pointers.values()];
      const distance = Math.hypot(values[0].x - values[1].x, values[0].y - values[1].y);
      const midpoint = this._localPoint({
        clientX: (values[0].x + values[1].x) / 2,
        clientY: (values[0].y + values[1].y) / 2,
      });
      if (this._pinchDistance) {
        this.zoomBy(distance / this._pinchDistance, midpoint.x, midpoint.y);
      }
      this._pinchDistance = distance;
      if (event && typeof event.preventDefault === "function") event.preventDefault();
    }
  }

  handlePointerUp(event) {
    const id = event && event.pointerId;
    if (id === undefined || id === null) return;
    const cancelled = Boolean(event && event.type === "pointercancel");
    const gesture = this._gesture && this._gesture.id === id ? this._gesture : null;
    const tap = Boolean(gesture && !gesture.moved && !cancelled && gesture.frame) &&
      this.pointers.size <= 1;
    this.pointers.delete(id);
    if (this.pointers.size < 2) this._pinchDistance = 0;
    if (gesture) {
      if (gesture.moved) this._suppressClick = true;
      if (this._capturedId === id) this._releasePointerCapture();
      this._gesture = null;
    }
    if (tap) {
      this._suppressClick = true;
      this.select(gesture.frame);
    }
  }

  handleClick(event) {
    if (!this.active) return;
    if (this._suppressClick) {
      this._suppressClick = false;
      return;
    }
    const frame = this._frameFromEvent(event);
    if (frame) this.select(frame);
  }

  handleKeyDown(event) {
    if (!this.active || !event) return;
    // transform.x/y are screen-space pan pixels, so the step must be a stable
    // screen distance rather than being divided by the zoom scale.
    const step = 48;
    switch (event.key) {
      case "ArrowUp":
        this.panBy(0, step);
        break;
      case "ArrowDown":
        this.panBy(0, -step);
        break;
      case "ArrowLeft":
        this.panBy(step, 0);
        break;
      case "ArrowRight":
        this.panBy(-step, 0);
        break;
      case "+":
      case "=":
        this.zoomBy(1.2);
        break;
      case "-":
      case "_":
        this.zoomBy(1 / 1.2);
        break;
      case "0":
        this.fit();
        break;
      default:
        return;
    }
    if (typeof event.preventDefault === "function") event.preventDefault();
  }

  handleResize() {
    const size = this.hostSize();
    const wasEmpty = this.lastWidth === 0 || this.lastHeight === 0;
    this.lastWidth = size.width;
    this.lastHeight = size.height;
    if (!size.width || !size.height) return;
    this._updateViewBox();
    if (this.needsFit || wasEmpty) this.fit();
    else this._applyTransform();
  }

  destroy() {
    this._releaseSceneListeners();
    for (const { target, type, handler, options } of this.hostListeners) {
      if (target && typeof target.removeEventListener === "function") {
        target.removeEventListener(type, handler, options);
      }
    }
    this.hostListeners = [];
    this._releasePointerCapture();
    if (this.resizeObserver && typeof this.resizeObserver.disconnect === "function") {
      this.resizeObserver.disconnect();
    }
    this.resizeObserver = null;
  }
}
