# Third-Party Notices

## shuding/liquid-glass (MIT)

The dynamic tab-track refraction generator in `static/track-refraction.js` is
adapted from [`shuding/liquid-glass`](https://github.com/shuding/liquid-glass)
("Copy-paste Liquid Glass effect using SVG filters"). The upstream repository
credits [`shuding/svg-shaders`](https://github.com/shuding/svg-shaders).

Adapted material:

- the `smoothStep` and `roundedRectSDF` helper functions;
- the per-pixel Canvas R/G displacement-map encoding and normalization;
- the SVG `feImage` + `feDisplacementMap` filter construction; and
- the `backdrop-filter: url(#id) ...` treatment.

The adaptation re-evaluates the SDF in CSS pixels for the wide `#viewTabs`
pill, regenerates the map at the real border-box size and current DPR, adds
`ResizeObserver`/window/DPR lifecycle, coalescing, generation guarding and
idempotent teardown, and uses the hyphenated `color-interpolation-filters`
attribute. The design is documented in
`docs/superpowers/specs/2026-09-23-dynamic-liquid-glass-tabs-design.md` and the
implementation plan in
`docs/superpowers/plans/2026-09-23-dynamic-liquid-glass-tabs.md`.

The static `LIQUID_DROP_MAP` asset, the `#liquidDrop` filter markup, the
capability helper, the stylesheet, and the transition state machine remain
independently implemented for this repository.

### Upstream license

MIT License

Copyright (c) 2025 Shu Ding

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
