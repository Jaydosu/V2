#!/usr/bin/env python3
"""
optimize_diode_offset.py -- find a better kbplacer diode placement offset
(the "diode_info.position" x/y/orientation field) for a diode-per-switch
keyboard matrix, by searching offsets that:

  1. keep the diode's B.CrtYd courtyard clear of the switch's B.CrtYd
     courtyard (with a clearance margin pulled from the actual project's
     netclass copper clearance, since KiCad doesn't define a separate
     default courtyard-clearance number), AND keep every individual diode
     pad's real copper footprint (size + rotation, not just its courtyard)
     clear of every individual switch pad -- courtyards on some hotswap
     footprints don't fully enclose every pad variant, so courtyard
     clearance alone is not sufficient to rule out physical pad overlap, and
  2. minimize the straight-line distance between the switch's row-side pin
     and the diode's matching pin (the pin on the intermediate
     "Net-(Dxx-A)"-style net) -- the trace kbplacer's autorouter has to
     draw for every single switch on the board.

Geometry is read directly from a real switch+diode footprint pair already
placed on YOUR board (not from a possibly-different copy of the library
footprint fetched from the internet) -- courtyard graphics and pad
positions are embedded per-instance in every .kicad_pcb footprint block,
so this uses your actual, current footprints.

Assumptions (stated explicitly, not silently):
  - the reference switch is mounted at rotation 0 (true for every switch
    checked on this board; if you have rotated keys, the switch-local frame
    used here won't match them -- rerun with a switch instance that has the
    stagger/rotation you care about)
  - only same-layer (both back-mounted, B.CrtYd vs B.CrtYd) courtyard overlap
    is checked, matching KiCad's own "courtyards overlap" DRC rule -- this
    script does not check copper-to-copper clearance beyond that, nor
    silkscreen/fab layers
  - courtyard clearance margin defaults to the project's Default netclass
    copper clearance (read from the .kicad_pro), since KiCad has no separate
    courtyard-clearance setting -- override with --clearance if you want a
    stricter/looser margin
  - the diode is assumed Back-mounted (matches every diode on this board);
    if yours are front-mounted, the F.Cu/B.Cu transform in
    transform_local() needs the mirror leg removed

Usage:
    python3 optimize_diode_offset.py V2.kicad_pcb --switch-ref SW87 --diode-ref D87 \
        --diode-net-pin-name auto --clearance 0.2

No third-party dependencies -- stdlib only.
"""
import argparse
import itertools
import json
import math
import re
import sys


def block_at(text, start):
    depth = 0
    i = start
    while True:
        if text[i] == '(':
            depth += 1
        elif text[i] == ')':
            depth -= 1
            if depth == 0:
                return text[start:i + 1]
        i += 1


def get_footprint_block(text, ref):
    idx = text.find(f'(property "Reference" "{ref}"')
    if idx == -1:
        raise SystemExit(f"Reference {ref!r} not found in board file")
    start = text.rfind('(footprint ', 0, idx)
    return block_at(text, start)


PAD_RE = re.compile(r'\(pad "([^"]*)" \S+ \S+\s*\(at ([^\)]+)\)(.*?)\n\s*\)\n', re.DOTALL)
NET_RE = re.compile(r'\(net "([^"]*)"\)')
LAYER_RE = re.compile(r'^\s*\(layer "([FB]\.Cu)"\)', re.MULTILINE)
FP_AT_RE = re.compile(r'\(footprint "[^"]*"\s*\n?\s*(?:\(layer[^\)]*\)\s*)?.*?\(at ([\-\d\.]+) ([\-\d\.]+)(?: ([\-\d\.]+))?\)', re.DOTALL)


def get_pad_records(block):
    """[(num, net, local_x, local_y, local_angle, w, h), ...] -- every physical
    pad instance on this footprint, with its own shape size and its own local
    rotation (the 3rd number in a pad's `(at x y angle)`, independent of the
    footprint's overall placement angle). Used to check actual copper-to-copper
    overlap, not just courtyard-box overlap -- courtyards on some hotswap
    footprints don't fully enclose every pad variant, so courtyard clearance
    alone is not sufficient to guarantee pads don't collide."""
    records = []
    for pm in PAD_RE.finditer(block):
        num = pm.group(1)
        at_parts = pm.group(2).split()
        px, py = float(at_parts[0]), float(at_parts[1])
        pangle = float(at_parts[2]) if len(at_parts) > 2 else 0.0
        rest = pm.group(3)
        size_m = re.search(r'\(size ([\-\d\.]+) ([\-\d\.]+)\)', rest)
        if not size_m:
            continue
        w, h = float(size_m.group(1)), float(size_m.group(2))
        net_m = NET_RE.search(rest)
        net = net_m.group(1) if net_m else None
        records.append((num, net, px, py, pangle, w, h))
    return records


def pad_local_corners(px, py, pangle, w, h):
    """4 corners of a pad's bounding rectangle, in the footprint's own local
    (pre-placement) frame. Treats roundrect/rect/circle/oval pads all as their
    w x h bounding rectangle -- a conservative (slightly oversized for circle/
    oval) approximation, which is the safe direction for a collision check."""
    hw, hh = w / 2, h / 2
    base = [(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)]
    theta = math.radians(pangle)
    ca, sa = math.cos(theta), math.sin(theta)
    corners = []
    for cx, cy in base:
        rx = cx * ca - cy * sa
        ry = cx * sa + cy * ca
        corners.append((px + rx, py + ry))
    return corners


def pad_board_boxes(block, dx, dy, angle_deg, back_mounted):
    """{pad_num: (net, board-frame bbox)} for every pad on this footprint,
    placed via the same transform_local() convention used for courtyard
    points (dx,dy = footprint origin, angle_deg = footprint placement angle)."""
    out = []
    for num, net, px, py, pangle, w, h in get_pad_records(block):
        corners_local = pad_local_corners(px, py, pangle, w, h)
        corners_board = transform_points(corners_local, dx, dy, angle_deg, back_mounted)
        out.append((num, net, bbox(corners_board)))
    return out


def get_pads(block):
    """{pad_number: [(local_x, local_y, net), ...]} -- list because hotswap
    footprints often have 2 physical pad instances (SMD + thru_hole) sharing
    one logical pad number."""
    pads = {}
    for pm in PAD_RE.finditer(block):
        num = pm.group(1)
        at_parts = pm.group(2).split()
        lx, ly = float(at_parts[0]), float(at_parts[1])
        nm = NET_RE.search(pm.group(3))
        net = nm.group(1) if nm else None
        pads.setdefault(num, []).append((lx, ly, net))
    return pads


def get_footprint_placement(block):
    m = re.search(r'\(footprint "[^"]*"[^\n]*\n\s*\(layer "([FB]\.Cu)"\)', block)
    layer = m.group(1) if m else None
    m2 = re.search(r'\(at ([\-\d\.]+) ([\-\d\.]+)(?: ([\-\d\.]+))?\)', block)
    x, y = float(m2.group(1)), float(m2.group(2))
    angle = float(m2.group(3)) if m2.group(3) else 0.0
    return x, y, angle, layer


def get_courtyard_points(block, layer):
    """All (x,y) vertices of fp_line/fp_rect/fp_poly graphics on `layer`,
    in the footprint's own LOCAL (pre-transform) frame."""
    pts = []
    for kind in ('fp_line', 'fp_rect'):
        for m in re.finditer(r'\(' + kind + r'\s', block):
            b = block_at(block, m.start())
            if f'"{layer}"' not in b:
                continue
            for xm in re.finditer(r'\((?:start|end)\s+([\-\d\.]+)\s+([\-\d\.]+)\)', b):
                pts.append((float(xm.group(1)), float(xm.group(2))))
    for m in re.finditer(r'\(fp_poly\s', block):
        b = block_at(block, m.start())
        if f'"{layer}"' not in b:
            continue
        for xm in re.finditer(r'\(xy ([\-\d\.]+) ([\-\d\.]+)\)', b):
            pts.append((float(xm.group(1)), float(xm.group(2))))
    return pts


def bbox(points):
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    return (min(xs), min(ys), max(xs), max(ys))


def transform_local(x, y, dx, dy, angle_deg, back_mounted):
    """Local footprint-frame point -> board frame, offset by (dx,dy) from
    the reference origin. Uses the same F.Cu/B.Cu convention as this
    project's kicad-schematic-check skill (Gotcha 4):
      F.Cu: rotate by -angle (KiCad's positive angle is CW in board coords)
      B.Cu: mirror X first, then rotate by +angle (not negated)
    """
    if back_mounted:
        x = -x
        theta = math.radians(angle_deg)
    else:
        theta = math.radians(-angle_deg)
    rx = x * math.cos(theta) - y * math.sin(theta)
    ry = x * math.sin(theta) + y * math.cos(theta)
    return (rx + dx, ry + dy)


def transform_points(points, dx, dy, angle_deg, back_mounted):
    return [transform_local(x, y, dx, dy, angle_deg, back_mounted) for x, y in points]


def bbox_overlap(b1, b2, margin):
    """True if two (xmin,ymin,xmax,ymax) boxes are within `margin` of each
    other (i.e. violate a required clearance gap)."""
    x1min, y1min, x1max, y1max = b1
    x2min, y2min, x2max, y2max = b2
    # expand box2 by margin on all sides, then test ordinary AABB overlap
    x2min -= margin; y2min -= margin; x2max += margin; y2max += margin
    return not (x1max < x2min or x2max < x1min or y1max < y2min or y2max < y1min)


def load_default_clearance(pro_path):
    if not pro_path:
        return None
    try:
        d = json.load(open(pro_path))
        classes = d.get('net_settings', {}).get('classes', [])
        for c in classes:
            if c.get('name') == 'Default':
                return c.get('clearance')
    except Exception:
        pass
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('board', help='.kicad_pcb file')
    ap.add_argument('--switch-ref', required=True, help='Reference switch instance to use as the geometry template, e.g. SW87')
    ap.add_argument('--diode-ref', required=True, help='Reference diode instance to use as the geometry template, e.g. D87')
    ap.add_argument('--pro', help='.kicad_pro file to pull the Default netclass clearance from (used as courtyard margin unless --clearance given)')
    ap.add_argument('--clearance', type=float, default=None, help='Courtyard clearance margin in mm (default: Default netclass clearance from --pro, or 0.2mm if unavailable)')
    ap.add_argument('--x-range', type=float, nargs=2, default=(-10, 12), help='dx search range in mm (default: -10 12)')
    ap.add_argument('--y-range', type=float, nargs=2, default=(-10, 10), help='dy search range in mm (default: -10 10)')
    ap.add_argument('--step', type=float, default=0.1, help='grid step in mm (default: 0.1)')
    ap.add_argument('--orientations', type=float, nargs='+', default=[0, 90, 180, 270], help='diode orientations to try in degrees (default: 0 90 180 270)')
    ap.add_argument('--top', type=int, default=10, help='how many best candidates to print (default: 10)')
    ap.add_argument('--no-widen', action='store_true',
                     help="constrain the search so the diode's footprint never extends further in +X than the "
                          "switch's own body (F.CrtYd) already does -- use this for an edge column where pushing "
                          "the diode further out would force the board itself wider")
    ap.add_argument('--max-x', type=float, default=None,
                     help='explicit cap (mm, switch-local frame) on how far right (+X) the diode footprint may '
                          'extend -- overrides --no-widen if both given')
    ap.add_argument('--check', type=float, nargs=3, metavar=('X', 'Y', 'ORIENTATION'),
                     help='skip the search -- just report whether this exact offset (dx dy orientation) passes '
                          'the courtyard-clearance and pad-overlap checks against this switch/diode pair')
    args = ap.parse_args()

    text = open(args.board, encoding='utf-8').read()
    sw_block = get_footprint_block(text, args.switch_ref)
    d_block = get_footprint_block(text, args.diode_ref)

    sw_x, sw_y, sw_angle, sw_layer = get_footprint_placement(sw_block)
    d_x, d_y, d_angle, d_layer = get_footprint_placement(d_block)
    if sw_angle != 0:
        print(f"WARNING: {args.switch_ref} is mounted at angle {sw_angle}, not 0 -- "
              "this script's switch-local frame assumes 0. Results will be wrong "
              "for boards where this differs from your target switches.", file=sys.stderr)

    sw_back = (sw_layer == 'B.Cu')
    d_back = (d_layer == 'B.Cu')
    print(f"{args.switch_ref}: layer={sw_layer} angle={sw_angle}    {args.diode_ref}: layer={d_layer} angle={d_angle} (current offset from switch: "
          f"({d_x - sw_x:.3f}, {d_y - sw_y:.3f}))")

    # courtyard on the layer that matters for a same-side collision check:
    # both footprints' overall B.CrtYd (or F.CrtYd) graphics, matching
    # whichever side the DIODE is mounted on (since that's the side we're
    # searching offsets on).
    crtyd_layer = 'B.CrtYd' if d_back else 'F.CrtYd'
    sw_crtyd_local = get_courtyard_points(sw_block, crtyd_layer)
    d_crtyd_local = get_courtyard_points(d_block, crtyd_layer)
    if not sw_crtyd_local:
        print(f"WARNING: no {crtyd_layer} graphics found on {args.switch_ref} -- "
              "collision checking against the switch will be skipped (unsafe).", file=sys.stderr)
    if not d_crtyd_local:
        raise SystemExit(f"No {crtyd_layer} graphics found on {args.diode_ref} -- can't check clearance.")

    # switch courtyard bbox in switch-local frame (switch treated as fixed at origin, angle sw_angle)
    sw_crtyd_pts = transform_points(sw_crtyd_local, 0, 0, sw_angle, sw_back) if sw_crtyd_local else []
    sw_crtyd_bbox = bbox(sw_crtyd_pts) if sw_crtyd_pts else None

    # the switch's own full body outline (F.CrtYd, since switches mount on F.Cu on this board) --
    # separate from sw_crtyd_bbox above, which may be the diode-side (B.CrtYd) courtyard instead
    # and on some hotswap footprints is a small L-shaped region, not the real housing outline.
    # This is the reference used for --no-widen/--max-x: "don't stick out past the switch itself."
    sw_body_layer = 'B.CrtYd' if sw_back else 'F.CrtYd'
    sw_body_local = get_courtyard_points(sw_block, sw_body_layer)
    sw_body_pts = transform_points(sw_body_local, 0, 0, sw_angle, sw_back) if sw_body_local else []
    sw_body_bbox = bbox(sw_body_pts) if sw_body_pts else None

    max_x = args.max_x
    if max_x is None and args.no_widen:
        if sw_body_bbox is None:
            raise SystemExit(f"--no-widen needs {args.switch_ref}'s {sw_body_layer} outline to know its extent, "
                              "but none was found -- pass --max-x explicitly instead.")
        max_x = sw_body_bbox[2]  # xmax
        print(f"--no-widen: capping diode extent to x <= {max_x:.3f} (switch's own {sw_body_layer} right edge)")

    # switch's row-side pin (the pin sharing the diode's "Net-(Dxx-A)"-style intermediate net) --
    # auto-detect: the switch pad number whose net matches the diode's non-COL-pattern pin's net.
    sw_pads = get_pads(sw_block)
    d_pads = get_pads(d_block)

    # figure out which diode pad is the "intermediate" (switch-side) pin: the one that
    # shares a net string with some switch pad.
    sw_nets = {net for plist in sw_pads.values() for (_, _, net) in plist}
    d_intermediate_pad = d_col_pad = None
    for num, plist in d_pads.items():
        net = plist[0][2]
        if net in sw_nets:
            d_intermediate_pad = num
        else:
            d_col_pad = num
    if d_intermediate_pad is None:
        raise SystemExit(f"Could not find a diode pad on {args.diode_ref} sharing a net with {args.switch_ref} -- "
                          "are you sure these two are actually connected?")

    # matching switch pad + prefer the SMD variant (same layer as a back-mounted diode) if there
    # are multiple physical instances of that logical pad (hotswap + through-hole combo footprints).
    target_net = d_pads[d_intermediate_pad][0][2]
    sw_pin_candidates = [(lx, ly) for plist in sw_pads.values() for (lx, ly, net) in plist if net == target_net]
    # crude SMD-preference heuristic: within a pad's variants, the one whose local x/y differs
    # from the through-hole default -- since we don't have per-pad layer info parsed here, just
    # report all candidates and use the one closest to the diode's current position as the
    # "preferred" one, matching what kbplacer's autorouter picked in the log.
    sw_pin_local = min(sw_pin_candidates, key=lambda p: math.hypot(p[0] - (d_x - sw_x), p[1] - (d_y - sw_y)))
    print(f"switch pin (net {target_net!r}) local position: ({sw_pin_local[0]:.3f}, {sw_pin_local[1]:.3f})")

    d_pin_local = d_pads[d_intermediate_pad][0][:2]
    d_col_local = d_pads[d_col_pad][0][:2] if d_col_pad else None
    print(f"diode intermediate pin (pad {d_intermediate_pad}) local position: {d_pin_local}")
    if d_col_pad:
        print(f"diode column-side pin (pad {d_col_pad}, net {d_pads[d_col_pad][0][2]!r}) local position: {d_col_local}")

    clearance = args.clearance
    if clearance is None:
        clearance = load_default_clearance(args.pro)
        if clearance is None:
            clearance = 0.2
            print("NOTE: no --pro given / no Default netclass clearance found -- using 0.2mm courtyard margin.", file=sys.stderr)
        else:
            print(f"Using Default netclass clearance from project as courtyard margin: {clearance}mm")

    # baseline (current) offset, for comparison
    base_dx, base_dy = d_x - sw_x, d_y - sw_y
    base_d_pin_global = transform_local(*d_pin_local, base_dx, base_dy, d_angle, d_back)
    base_dist = math.hypot(base_d_pin_global[0] - sw_pin_local[0], base_d_pin_global[1] - sw_pin_local[1])
    print(f"\nCurrent offset ({base_dx:.3f}, {base_dy:.3f}) @ {d_angle} deg -> "
          f"switch-to-diode trace distance = {base_dist:.3f}mm\n")

    xs = frange(args.x_range[0], args.x_range[1], args.step)
    ys = frange(args.y_range[0], args.y_range[1], args.step)

    # real copper: every physical pad on both footprints, in their own placed
    # (board-frame) positions. Switch is fixed (dx=dy=0, its own angle/side);
    # diode pads get re-transformed per-candidate below. A pad-to-pad overlap
    # is a hard reject regardless of courtyard clearance, because courtyards
    # on some hotswap footprints don't fully enclose every pad variant (this
    # was confirmed on this exact board: SW87's B.CrtYd polygon does NOT
    # cover its own SMD pad 2 at local (5.32,-5.08), so courtyard-only
    # checking let a candidate through that overlapped that pad).
    sw_pad_boxes = pad_board_boxes(sw_block, 0, 0, sw_angle, sw_back)
    # require at least the project's own copper clearance as a real edge-to-edge gap between
    # any two pads (same value used for courtyard margin above -- pads on the same net don't
    # strictly need this by DRC, but leaving real solder/manufacturing room is good practice
    # regardless, and it's the same number already loaded from the project)
    pad_gap_min = clearance

    def pads_collide(d_pad_boxes):
        for _, _, dbox in d_pad_boxes:
            for _, _, sbox in sw_pad_boxes:
                if bbox_overlap(sbox, dbox, pad_gap_min):
                    return True
        return False

    if args.check is not None:
        cx, cy, cangle = args.check
        print(f"--check: evaluating offset ({cx}, {cy}) @ {cangle} deg against {args.switch_ref}/{args.diode_ref}\n")
        ok = True

        d_crtyd_cand = transform_points(d_crtyd_local, cx, cy, cangle, d_back)
        d_crtyd_cand_bbox = bbox(d_crtyd_cand)
        if sw_crtyd_bbox:
            crtyd_bad = bbox_overlap(sw_crtyd_bbox, d_crtyd_cand_bbox, clearance)
            print(f"courtyard-vs-courtyard ({clearance}mm margin): "
                  f"{'FAIL -- too close / overlapping' if crtyd_bad else 'OK'}")
            ok = ok and not crtyd_bad
        else:
            print(f"courtyard-vs-courtyard: SKIPPED -- no {crtyd_layer} graphics found on "
                  f"{args.switch_ref}, can't check (this is the case for footprints like a rotary "
                  f"encoder switch -- treat this offset as unverified for that part, check visually)")

        d_pad_boxes_cand = pad_board_boxes(d_block, cx, cy, cangle, d_back)
        pad_hits = []
        for dn, dnet, dbox_ in d_pad_boxes_cand:
            for sn, snet, sbox in sw_pad_boxes:
                if bbox_overlap(sbox, dbox_, pad_gap_min):
                    pad_hits.append((dn, dnet, sn, snet))
        if pad_hits:
            print(f"pad-vs-pad ({pad_gap_min}mm gap required): FAIL -- overlaps:")
            for dn, dnet, sn, snet in pad_hits:
                print(f"    diode pad {dn} (net {dnet!r}) vs switch pad {sn} (net {snet!r})")
        else:
            print(f"pad-vs-pad ({pad_gap_min}mm gap required): OK -- no overlap with any switch pad "
                  f"(checked {len(sw_pad_boxes)} switch pads incl. mechanical/unlabeled ones)")
        ok = ok and not pad_hits

        if max_x is not None:
            x_bad = d_crtyd_cand_bbox[2] > max_x
            print(f"max-x constraint (<= {max_x:.3f}mm): "
                  f"{'FAIL -- extends to ' + format(d_crtyd_cand_bbox[2], '.3f') + 'mm' if x_bad else 'OK'}")
            ok = ok and not x_bad

        d_pin_cand = transform_local(*d_pin_local, cx, cy, cangle, d_back)
        dist = math.hypot(d_pin_cand[0] - sw_pin_local[0], d_pin_cand[1] - sw_pin_local[1])
        print(f"\nswitch-to-diode trace distance at this offset: {dist:.3f}mm (current board offset gives "
              f"{base_dist:.3f}mm)")

        print(f"\nOverall: {'PASSES' if ok else 'FAILS'} the checks this script can run.")
        if crtyd_layer == ('B.CrtYd' if d_back else 'F.CrtYd') and not sw_crtyd_bbox:
            print("(courtyard check was skipped -- see note above -- so this is pad-collision-verified only)")
        return

    results = []
    rejected_by_pad_check = 0
    rejected_by_max_x = 0
    for angle in args.orientations:
        d_crtyd_at_origin = transform_points(d_crtyd_local, 0, 0, angle, d_back)
        d_pin_at_origin = transform_local(*d_pin_local, 0, 0, angle, d_back)
        for dx, dy in itertools.product(xs, ys):
            # cheap reject: bbox at this dx,dy vs switch bbox, using precomputed origin-relative size
            d_bbox = (dx + min(p[0] for p in d_crtyd_at_origin), dy + min(p[1] for p in d_crtyd_at_origin),
                      dx + max(p[0] for p in d_crtyd_at_origin), dy + max(p[1] for p in d_crtyd_at_origin))
            if max_x is not None and d_bbox[2] > max_x:
                rejected_by_max_x += 1
                continue
            if sw_crtyd_bbox and bbox_overlap(sw_crtyd_bbox, d_bbox, clearance):
                continue
            # real pad-vs-pad check -- courtyard clearance alone is not sufficient (see above)
            d_pad_boxes = pad_board_boxes(d_block, dx, dy, angle, d_back)
            if pads_collide(d_pad_boxes):
                rejected_by_pad_check += 1
                continue
            pin_global = (d_pin_at_origin[0] + dx, d_pin_at_origin[1] + dy)
            dist = math.hypot(pin_global[0] - sw_pin_local[0], pin_global[1] - sw_pin_local[1])
            results.append((dist, dx, dy, angle))

    print(f"(pad-vs-pad check rejected {rejected_by_pad_check} candidates that cleared the courtyard check "
          f"but would have physically overlapped a switch pad)")
    if max_x is not None:
        print(f"(max-x constraint rejected {rejected_by_max_x} candidates that would have extended past x={max_x:.3f})")

    if not results:
        print("No candidate offsets cleared both the courtyard-clearance AND pad-overlap checks in this "
              "search range -- widen --x-range/--y-range or reduce --clearance.")
        return

    results.sort(key=lambda r: r[0])

    def col_shift_for(dx, dy, angle):
        if not d_col_local:
            return None
        base_col_global = transform_local(*d_col_local, base_dx, base_dy, d_angle, d_back)
        cand_col_global = transform_local(*d_col_local, dx, dy, angle, d_back)
        return math.hypot(cand_col_global[0] - base_col_global[0], cand_col_global[1] - base_col_global[1])

    print(f"{'rank':4} {'dist(mm)':9} {'dx':7} {'dy':7} {'orient':7} {'vs current':11} {'col-pin shift':13}")
    print('-' * 70)
    for i, (dist, dx, dy, angle) in enumerate(results[:args.top], 1):
        delta = dist - base_dist
        cshift = col_shift_for(dx, dy, angle)
        cshift_str = f"{cshift:.2f}mm" if cshift is not None else "n/a"
        print(f"{i:<4} {dist:<9.3f} {dx:<7.2f} {dy:<7.2f} {angle:<7.0f} {delta:<+11.3f} {cshift_str}")

    print("\nNOTE on 'col-pin shift': this is how far the diode's column-side pin moves relative to")
    print("THIS switch. Since kbplacer applies one diode_info offset to every switch on the board,")
    print("every diode's column pin shifts by roughly the same vector -- for keys in a straight,")
    print("unrotated column this mostly cancels out (the whole column bus translates together, so")
    print("trace length between consecutive diodes barely changes). It does NOT cancel out for keys")
    print("whose rotation differs from their neighbors (stagger/rotary-encoder keys) -- re-run this")
    print("script with --switch-ref/--diode-ref pointed at one of those to check them specifically.")

    best = results[0]
    print(f"\nBest (shortest switch-to-diode trace): offset=({best[1]:.2f}, {best[2]:.2f}) orientation={best[3]:.0f} deg")
    print(f"  -> switch-to-diode trace distance {best[0]:.3f}mm (was {base_dist:.3f}mm, "
          f"{'shorter' if best[0] < base_dist else 'longer'} by {abs(best[0]-base_dist):.3f}mm)")
    print(f"\nSet this in kbplacer's diode_info: position_option=Custom, "
          f"x={best[1]:.2f}, y={best[2]:.2f}, orientation={best[3]:.0f}, side={'Back' if d_back else 'Front'}")


def frange(start, stop, step):
    n = int(round((stop - start) / step))
    return [start + i * step for i in range(n + 1)]


if __name__ == '__main__':
    main()
