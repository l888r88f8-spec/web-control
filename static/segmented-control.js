// Reusable pointer-drag controller for segmented controls.
//
// The sliding capsule is positioned from measured pixels instead of a
// percentage of its own width. That matters because the track is a
// shrink-to-fit grid whose columns need not be equal, and because
// `*{box-sizing:border-box}` does not reach pseudo-elements: translating the
// capsule by `100%` per step also carried its own 2px of border, so the
// rightmost segment drifted past the track. Measuring each button keeps the
// capsule exactly inside the inner track at any width or reflow.

export const SEGMENT_DRAG_THRESHOLD_PX = 4;
export const THUMB_DRAGGING_CLASS = "thumb-dragging";
const THUMB_X_PROP = "--thumb-x";
const THUMB_W_PROP = "--thumb-w";
const CLICK_SUPPRESS_MS = 400;

export function clamp(value, min, max) {
  if (max < min) return min;
  if (value < min) return min;
  if (value > max) return max;
  return value;
}

// `trackRect` and each `buttonRect` are getBoundingClientRect() results.
// `borderLeft`/`borderRight` are the track's computed border widths. Capsule x
// is measured from the track's padding edge, which is the containing block for
// the absolutely positioned `::before` capsule.
export function buildSegmentLayout(trackRect, buttonRects, borderLeft = 0, borderRight = 0) {
  const innerWidth = Math.max(0, trackRect.width - borderLeft - borderRight);
  const contentLeft = trackRect.left + borderLeft;
  const segments = Array.from(buttonRects, (rect) => {
    const width = rect.width;
    const x = clamp(rect.left - contentLeft, 0, Math.max(0, innerWidth - width));
    return { x, width };
  });
  return { innerWidth, segments };
}

export function thumbXForIndex(layout, index) {
  const segment = layout.segments[clamp(index, 0, layout.segments.length - 1)];
  if (!segment) return 0;
  return clamp(segment.x, 0, Math.max(0, layout.innerWidth - segment.width));
}

export function nearestSegmentIndex(layout, centerX) {
  const segments = layout.segments;
  if (!segments.length) return 0;
  let bestIndex = 0;
  let bestDistance = Infinity;
  segments.forEach((segment, index) => {
    const distance = Math.abs(segment.x + segment.width / 2 - centerX);
    if (distance < bestDistance) {
      bestDistance = distance;
      bestIndex = index;
    }
  });
  return bestIndex;
}

export function dragThumbX(layout, width, x) {
  return clamp(x, 0, Math.max(0, layout.innerWidth - width));
}

function readBorderWidth(track, side) {
  try {
    if (typeof window === "undefined" || typeof window.getComputedStyle !== "function") return 0;
    const styles = window.getComputedStyle(track);
    if (!styles) return 0;
    const value = parseFloat(styles[`border${side}Width`]);
    return Number.isFinite(value) ? value : 0;
  } catch (_error) {
    return 0;
  }
}

export function measureSegmentLayout(track, buttons) {
  const trackRect = track.getBoundingClientRect();
  const borderLeft = readBorderWidth(track, "Left");
  const borderRight = readBorderWidth(track, "Right");
  const buttonRects = buttons.map((button) => button.getBoundingClientRect());
  return buildSegmentLayout(trackRect, buttonRects, borderLeft, borderRight);
}

export function createSegmentedControl(options = {}) {
  const track = options.track;
  const buttons = Array.from(options.buttons || []);
  const noop = () => {};

  if (!track || !buttons.length) {
    return { refresh: noop, setActive: noop, sync: noop, destroy: noop };
  }

  const threshold = Number.isFinite(options.threshold)
    ? options.threshold
    : SEGMENT_DRAG_THRESHOLD_PX;
  const onSelect = typeof options.onSelect === "function" ? options.onSelect : noop;
  const getActiveIndex =
    typeof options.getActiveIndex === "function"
      ? options.getActiveIndex
      : () => {
          const index = buttons.findIndex(
            (button) => button.classList && button.classList.contains("active"),
          );
          return index >= 0 ? index : 0;
        };

  let layout = null;
  let drag = null;
  let suppressClick = false;
  let suppressTimerId = null;
  let resizeObserver = null;
  let windowResizeListener = false;

  function readIndex() {
    return clamp(getActiveIndex(), 0, buttons.length - 1);
  }

  function layoutSegment(index) {
    return layout ? layout.segments[clamp(index, 0, layout.segments.length - 1)] : null;
  }

  function applyThumb(x, width, animate) {
    if (!animate) track.classList.add(THUMB_DRAGGING_CLASS);
    track.style.setProperty(THUMB_X_PROP, `${x}px`);
    if (Number.isFinite(width)) track.style.setProperty(THUMB_W_PROP, `${width}px`);
    if (!animate) {
      // Flush the reposition while transitions are off so a later resize does
      // not animate from stale geometry.
      void track.offsetWidth;
      track.classList.remove(THUMB_DRAGGING_CLASS);
    }
  }

  function refresh() {
    layout = measureSegmentLayout(track, buttons);
    if (!(drag && drag.dragging)) {
      const index = readIndex();
      const segment = layoutSegment(index);
      applyThumb(thumbXForIndex(layout, index), segment ? segment.width : 0, false);
    }
    return layout;
  }

  function setActive(index) {
    if (!layout) layout = measureSegmentLayout(track, buttons);
    const target = clamp(index, 0, buttons.length - 1);
    const segment = layoutSegment(target);
    applyThumb(thumbXForIndex(layout, target), segment ? segment.width : 0, true);
  }

  function sync() {
    if (!layout) layout = measureSegmentLayout(track, buttons);
    const index = readIndex();
    const segment = layoutSegment(index);
    applyThumb(thumbXForIndex(layout, index), segment ? segment.width : 0, true);
  }

  function clearSuppressTimer() {
    if (suppressTimerId !== null && typeof window !== "undefined" && typeof window.clearTimeout === "function") {
      window.clearTimeout(suppressTimerId);
    }
    suppressTimerId = null;
  }

  function onPointerDown(event) {
    if (event.button !== undefined && event.button !== 0) return;
    if (drag) return;
    suppressClick = false;
    layout = measureSegmentLayout(track, buttons);
    const index = readIndex();
    const segment = layoutSegment(index);
    const baseX = thumbXForIndex(layout, index);
    drag = {
      pointerId: event.pointerId,
      startX: event.clientX,
      baseX,
      x: baseX,
      width: segment ? segment.width : 0,
      index,
      dragging: false,
    };
  }

  function onPointerMove(event) {
    if (!drag || event.pointerId !== drag.pointerId) return;
    const deltaX = event.clientX - drag.startX;
    if (!drag.dragging) {
      if (Math.abs(deltaX) < threshold) return;
      drag.dragging = true;
      track.classList.add(THUMB_DRAGGING_CLASS);
    }
    drag.x = dragThumbX(layout, drag.width, drag.baseX + deltaX);
    track.style.setProperty(THUMB_X_PROP, `${drag.x}px`);
    if (typeof event.preventDefault === "function") event.preventDefault();
  }

  function settle(cancelled) {
    const current = drag;
    if (!current) return;
    drag = null;
    if (!current.dragging) return;
    track.classList.remove(THUMB_DRAGGING_CLASS);
    if (cancelled) {
      setActive(current.index);
      return;
    }
    const index = nearestSegmentIndex(layout, current.x + current.width / 2);
    suppressClick = true;
    clearSuppressTimer();
    if (typeof window !== "undefined" && typeof window.setTimeout === "function") {
      suppressTimerId = window.setTimeout(() => {
        suppressClick = false;
        suppressTimerId = null;
      }, CLICK_SUPPRESS_MS);
    }
    setActive(index);
    onSelect(index, buttons[index]);
  }

  function onPointerUp(event) {
    if (!drag || event.pointerId !== drag.pointerId) return;
    settle(false);
  }

  function onPointerCancel(event) {
    if (!drag || (event && event.pointerId !== undefined && event.pointerId !== drag.pointerId)) return;
    settle(true);
  }

  function onLostPointerCapture() {
    if (!drag) return;
    settle(true);
  }

  function onClickCapture(event) {
    if (!suppressClick) return;
    suppressClick = false;
    if (typeof event.preventDefault === "function") event.preventDefault();
    if (typeof event.stopImmediatePropagation === "function") {
      event.stopImmediatePropagation();
    } else if (typeof event.stopPropagation === "function") {
      event.stopPropagation();
    }
  }

  function observeResize() {
    if (typeof ResizeObserver === "function") {
      resizeObserver = new ResizeObserver(() => refresh());
      resizeObserver.observe(track);
      return;
    }
    if (typeof window !== "undefined" && typeof window.addEventListener === "function") {
      window.addEventListener("resize", refresh);
      windowResizeListener = true;
    }
  }

  function destroy() {
    track.removeEventListener("pointerdown", onPointerDown);
    track.removeEventListener("lostpointercapture", onLostPointerCapture);
    track.removeEventListener("click", onClickCapture, true);
    gestureTarget.removeEventListener("pointermove", onPointerMove);
    gestureTarget.removeEventListener("pointerup", onPointerUp);
    gestureTarget.removeEventListener("pointercancel", onPointerCancel);
    if (resizeObserver) {
      resizeObserver.disconnect();
      resizeObserver = null;
    }
    if (windowResizeListener && typeof window !== "undefined") {
      window.removeEventListener("resize", refresh);
      windowResizeListener = false;
    }
    clearSuppressTimer();
  }

  // Pointer move/up/cancel live on window so a drag that leaves the track still
  // settles. Capturing the pointer on pointerdown instead would retarget the
  // synthesized click away from the button and break plain click selection.
  const gestureTarget =
    typeof window !== "undefined" && typeof window.addEventListener === "function"
      ? window
      : track;

  track.addEventListener("pointerdown", onPointerDown);
  track.addEventListener("lostpointercapture", onLostPointerCapture);
  track.addEventListener("click", onClickCapture, true);
  gestureTarget.addEventListener("pointermove", onPointerMove, { passive: false });
  gestureTarget.addEventListener("pointerup", onPointerUp);
  gestureTarget.addEventListener("pointercancel", onPointerCancel);

  refresh();
  observeResize();

  return { refresh, setActive, sync, destroy };
}
