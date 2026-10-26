import itertools, json, re, sys
P = "shelf.00.$WhichApollo"
DS = {"type": "graphite", "uid": "${DS}"}
panels = []
pid = [0]
y = [0]

def nid():
    pid[0] += 1
    return pid[0]

# Graphite averages points when a range has more of them than the panel is wide. That is
# right for temperatures, but turns states, codes, masks and raw words into meaningless
# fractions (e.g. power state 6.11), so those keep the last value of each merged bucket.
def consolidation(expr):
    if "consolidateBy" in expr or "TEMP1_C" in expr or "TEMP3_C" in expr:
        return expr
    func = "min" if "FREE_WORDS" in expr else "max" if "MCU_HEALTH" in expr else "last"
    return re.sub(r"(shelf\.00\.\$WhichApollo(?:[\w.*$]|\{[^}]*\})+)", r"consolidateBy(\1, '%s')" % func, expr)

# Grafana's Graphite query editor only parses a brace list that is a whole path element of
# complete names ({A_X,A_Y}), not text around or between braces (A_{X,Y}, B{1,3}_{X,Y}), so
# expand those into full names.
def expand_braces(expr):
    def node(m):
        parts = re.split(r"(\{[^}]*\})", m.group(0))
        opts = [p[1:-1].split(",") if p.startswith("{") else [p] for p in parts]
        names = ["".join(c) for c in itertools.product(*opts)]
        return names[0] if len(names) == 1 else "{" + ",".join(names) + "}"
    return re.sub(r"[\w*]*\{[^}]*\}(?:[\w*]|\{[^}]*\})*", node, expr)

def tgt(expr, ref="A"):
    return {"refId": ref, "target": consolidation(expand_braces(expr)), "datasource": DS}

def targets(exprs):
    return [tgt(e, chr(ord("A") + i)) for i, e in enumerate(exprs)]

def mapping(d):
    return [{"type": "value", "options": {str(k): {"text": v, "index": i} for i, (k, v) in enumerate(d.items())}}]

def thresholds(steps):
    return {"mode": "absolute", "steps": [{"color": c, "value": v} for v, c in steps]}

def stat(title, exprs, x, w, h=4, calc="lastNotNull", steps=((None, "green"),), unit="none",
         maps=None, desc=""):
    p = {"id": nid(), "type": "stat", "title": title, "description": desc, "datasource": DS,
         "gridPos": {"x": x, "y": y[0], "w": w, "h": h},
         "targets": targets(exprs),
         "options": {"reduceOptions": {"calcs": [calc], "fields": "", "values": False},
                     "colorMode": "background", "graphMode": "none", "textMode": "value_and_name",
                     "orientation": "auto", "justifyMode": "auto"},
         "fieldConfig": {"defaults": {"unit": unit, "thresholds": thresholds(steps),
                                      "mappings": mapping(maps) if maps else [],
                                      "color": {"mode": "thresholds"}}, "overrides": []}}
    return p

def ts(title, exprs, x, w, h=8, unit="none", desc="", stepped=False):
    custom = {"lineInterpolation": "stepAfter" if stepped else "linear", "spanNulls": True,
              "showPoints": "never", "fillOpacity": 0}
    return {"id": nid(), "type": "timeseries", "title": title, "description": desc, "datasource": DS,
            "gridPos": {"x": x, "y": y[0], "w": w, "h": h}, "targets": targets(exprs),
            "options": {"legend": {"displayMode": "list", "placement": "bottom", "showLegend": True},
                        "tooltip": {"mode": "multi", "sort": "none"}},
            "fieldConfig": {"defaults": {"unit": unit, "custom": custom, "color": {"mode": "palette-classic"}},
                            "overrides": []}}

def timeline(title, exprs, x, w, maps, h=6, desc=""):
    return {"id": nid(), "type": "state-timeline", "title": title, "description": desc, "datasource": DS,
            "gridPos": {"x": x, "y": y[0], "w": w, "h": h}, "targets": targets(exprs),
            "options": {"showValue": "auto", "mergeValues": True, "rowHeight": 0.9,
                        "legend": {"showLegend": False}},
            "fieldConfig": {"defaults": {"mappings": mapping(maps), "color": {"mode": "palette-classic"},
                                         "thresholds": thresholds([(None, "green")])}, "overrides": []}}

def row(title, collapsed, children):
    r = {"id": nid(), "type": "row", "title": title, "collapsed": collapsed,
         "gridPos": {"x": 0, "y": y[0], "w": 24, "h": 1}, "panels": []}
    y[0] += 1
    return r

# increase over the selected range, robust to the counters restarting at boot
def inc(path, alias_node):
    # consolidateBy sum: over long ranges Graphite merges points, and an average would undercount
    # keepLastValue: nonNegativeDerivative returns null next to any gap in the series, so a
    # flat counter with gaps (good condition) would otherwise come out as "No data"
    return f"aliasByNode(consolidateBy(nonNegativeDerivative(keepLastValue({P}.{path})), 'sum'), {alias_node})"

PWR = {0: "FAILURE", 1: "INIT", 2: "DOWN", 3: "OFF", 4: "L1ON", 5: "L2ON", 6: "L3ON", 7: "L4ON",
       8: "L5ON", 9: "L6ON", 10: "ON", 11: "CONFIG"}
STEP = dict(PWR); STEP[0] = "none"
FATAL = {0: "none", 7: "HARD FAULT", 8: "ASSERT", 9: "STACK OVERFLOW", 21: "MALLOC FAIL"}
I2CS_STATE = {0: "READY", 1: "FIRSTBYTE", 2: "RX_B0", 3: "RX_B1", 4: "TX_B0", 5: "TX_B1", 255: "none"}
TASKS = ["POW", "LED", "CLIZY", "ADC", "FF_F1", "FF_F2", "CLK", "PSMON", "XIMON", "I2CS0", "EPRM",
         "ZMON", "TALM", "VALM", "PRGCM"]
TASK = {i: t for i, t in enumerate(TASKS)}; TASK[254] = "other"; TASK[255] = "none"
RAILS = ["1V8", "3V3", "F1VCCINT1_S1", "F1VCCINT1_S2", "F1VCCINT2_S1", "F1VCCINT2_S2", "F2VCCINT1_S1",
         "F2VCCINT1_S2", "F2VCCINT2_S1", "F2VCCINT2_S2", "F1AVTT", "F1AVCC", "F2AVTT", "F2AVCC"]
RAIL = {i: r for i, r in enumerate(RAILS)}; RAIL[255] = "none"
RED_IF_ANY = ((None, "green"), (1, "red"))

# ---- indicators ----
panels.append(row("Indicators (alarm on these)", False, []))
panels += [
    stat("MCU health", [f"alias({P}.CM_DIAG.MCU_HEALTH.VALUE, 'health')"], 0, 3, steps=RED_IF_ANY,
         maps={0: "OK"}, desc="Non-zero: power fault, temperature or voltage alarm active now (see HEALTH_* bits)."),
    stat("Power state", [f"alias({P}.CM_DIAG_PWR.PWR_STATE.VALUE, 'state')"], 3, 3,
         steps=((None, "red"), (1, "blue"), (10, "green"), (11, "blue")), maps=PWR,
         desc="POWER_FAILURE is red."),
    stat("Previous boot ended in", [f"alias({P}.CM_DIAG_RESET.PREV_FATAL_CODE.VALUE, 'prev fatal')"], 6, 3,
         steps=RED_IF_ANY, maps=FATAL,
         desc="Newest fatal error-log entry of the previous boot. Suspect if RESET_CAUSE is not SW."),
    stat("Power failures in range", [inc("CM_DIAG_PWR.PWR_{SUPPLY_FAIL,ALARM_OFF,CONFIG_FAIL}.VALUE", 4)],
         9, 5, calc="sum", steps=RED_IF_ANY, desc="Increase of the PWR_* failure counters over the time range."),
    stat("I2C master errors in range", [inc("CM_DIAG_I2C.*.{START_FAIL,ERROR}", "4, 5")], 14, 5, calc="sum",
         steps=RED_IF_ANY, desc="Failed transfer starts and non-NACK errors per bus (NACKs are not counted)."),
    stat("Min free stack (words)", [f"alias(minSeries({P}.CM_DIAG_STACK.*.FREE_WORDS), 'min')"], 19, 3,
         calc="min", steps=((None, "red"), (32, "orange"), (64, "green")),
         desc="Lowest stack high-water mark over all tasks. Thresholds are a suggestion."),
    stat("MCU uptime", [f"alias({P}.CM_MON.MCU_UPTIME.MINUTES, 'uptime')"], 22, 2, unit="m",
         desc="Drops to 0 at a reset; all counters restart then."),
]
y[0] += 4
panels += [
    stat("I2C slave (IPMC) events in range", [inc("CM_DIAG_I2CS.I2CS_{DUP_IRQ,UNHANDLED}.VALUE", 4)], 0, 6,
         calc="sum", steps=RED_IF_ANY, desc="Duplicate DATA interrupts and unhandled I2CSCSR values."),
    stat("I2C slave timeouts in range", [inc("CM_DIAG_I2CS.I2CS_TIMEOUTS.VALUE", 4)], 6, 3, calc="sum",
         steps=((None, "green"), (10, "orange"), (100, "red")),
         desc="Also counts harmless lone 1-byte writes, so only a fast rise matters."),
    stat("LGA80D bad temperature reads in range", [inc("CM_DIAG_LGA80D.LGA80D_BADT{1,3}_{FFFF,OTHER}.VALUE", 4)],
         9, 10, calc="sum", steps=RED_IF_ANY,
         desc="READ_TEMPERATURE_1 (BADT1) / _3 (BADT3) words outside 0..175 C; FFFF = the -0.50 C glitch."),
    stat("Diag schema", [f"alias({P}.CM_DIAG.DIAG_SCHEMA.VALUE, 'schema')"], 19, 5,
         steps=((None, "red"), (8, "green"), (9, "red")),
         desc="This dashboard was built for DIAG_SCHEMA 8."),
]
y[0] += 4

# ---- diagnostics: power ----
r = row("Diagnostics: power", True, [])
y0 = y[0]
r["panels"] += [
    timeline("Power state", [f"alias({P}.CM_DIAG_PWR.PWR_STATE.VALUE, 'state')",
                             f"alias({P}.CM_DIAG_PWR.PWR_FAIL_STEP.VALUE, 'last failure entered from')"],
             0, 12, PWR, desc="PWR_FAIL_STEP 0 means no failure since boot."),
    stat("Supply masks", [f"aliasByNode({P}.CM_DIAG_PWR.PWR_{{LIVE,EXPECTED,IGNORE,FAILED}}_MASK.VALUE, 4)"], 12, 12,
         h=6, unit="hex0x", desc="One bit per supply (PS_OKS order). FAILED_MASK says which supplies failed."),
]
y[0] += 6
r["panels"] += [
    stat("Power state flags", [f"aliasByNode({P}.CM_DIAG_PWR.PWR_{{BLADE_POWER_EN,CLI_INHIBIT,PROGCOM_INHIBIT,FAULT_LATCH,ALARM_SHUTDOWN_LATCH,F1_ENABLE,F2_ENABLE}}.VALUE, 4)"],
         0, 24, h=4, steps=((None, "text"),)),
]
y[0] += 4
r["panels"] += [
    ts("Power failure counters (since boot)", [f"aliasByNode({P}.CM_DIAG_PWR.PWR_{{SUPPLY_FAIL,ALARM_OFF,CONFIG_FAIL}}.VALUE, 4)"],
       0, 12, stepped=True),
    ts("Snapshot generation", [f"alias({P}.CM_DIAG_PWR.PWR_GENERATION.VALUE, 'generation')"], 12, 12, stepped=True,
       desc="Increments by 2 per power snapshot change."),
]
y[0] += 8
panels.append(r)

# ---- diagnostics: reset ----
r = row("Diagnostics: reset", True, [])
r["panels"] += [
    stat("Previous boot fatal entry", [f"alias({P}.CM_DIAG_RESET.PREV_FATAL_CODE.VALUE, 'code')",
                                       f"alias({P}.CM_DIAG_RESET.PREV_FATAL_DATA.VALUE, 'data')",
                                       f"alias({P}.CM_DIAG_RESET.PREV_FATAL_ENTRY.VALUE, 'entry')"], 0, 8,
         unit="hex0x", desc="Error-log entry: code (7 hard fault, 8 assert, 9 stack overflow, 21 malloc) and data."),
    stat("Assert PC", [f"alias({P}.CM_DIAG_RESET.PREV_ASSERT_PC.VALUE, 'PC')"], 8, 4, unit="hex0x",
         desc="Decode with arm-none-eabi-addr2line -e cm_mcu.axf <PC> for the matching build."),
    stat("Reset cause", [f"alias({P}.CM_DIAG_RESET.RESET_CAUSE.VALUE, 'RESC')",
                         f"aliasByNode({P}.CM_DIAG_RESET.RESET_{{POR,EXT,SW,BOR,WDOG0,WDOG1,HIB,HSRVREQ}}.VALUE, 4)"],
         12, 12, unit="none", steps=((None, "text"),)),
]
y[0] += 4
panels.append(r)

# ---- diagnostics: I2C ----
r = row("Diagnostics: I2C", True, [])
r["panels"] += [
    ts("I2C master counters (since boot)", [f"aliasByNode({P}.CM_DIAG_I2C.*.{{START_FAIL,ERROR}}, 4, 5)"], 0, 12,
       stepped=True),
    ts("I2C slave counters (since boot)", [f"aliasByNode({P}.CM_DIAG_I2CS.I2CS_{{DUP_IRQ,TIMEOUTS,UNHANDLED}}.VALUE, 4)"],
       12, 12, stepped=True),
]
y[0] += 8
r["panels"] += [
    stat("I2C slave registers", [f"aliasByNode({P}.CM_DIAG_I2CS.I2CS_{{LAST_UNHANDLED,SNAP_INT,SNAP_SCSR}}.VALUE, 4)"],
         0, 12, unit="hex0x", desc="SNAP_* describe the first duplicate interrupt (meaningful once DUP_IRQ > 0)."),
    stat("First duplicate: task state", [f"alias({P}.CM_DIAG_I2CS.I2CS_SNAP_STATE.VALUE, 'state')"], 12, 6,
         maps=I2CS_STATE),
    stat("First duplicate: interrupted task", [f"alias({P}.CM_DIAG_I2CS.I2CS_SNAP_TASK.VALUE, 'task')"], 18, 6,
         maps=TASK),
]
y[0] += 4
panels.append(r)

# ---- diagnostics: LGA80D ----
r = row("Diagnostics: LGA80D", True, [])
for i, s in enumerate(("1", "3")):
    r["panels"] += [
        stat(f"TEMP{s}: latest bad word", [f"alias({P}.CM_DIAG_LGA80D.LGA80D_BADT{s}_RAW.VALUE, 'raw')"], 12 * i, 6,
             unit="hex0x", desc="0 until this sensor's first bad read."),
        stat(f"TEMP{s}: latest bad rail", [f"alias({P}.CM_DIAG_LGA80D.LGA80D_BADT{s}_ROW.VALUE, 'rail')"], 12 * i + 6, 6,
             maps=RAIL),
    ]
y[0] += 4
r["panels"] += [
    ts("LGA80D TEMP1 (controller)", [f"aliasByNode({P}.CM_MON.*.TEMP1_C, 4)"], 0, 12, unit="celsius"),
    ts("LGA80D TEMP3 (power stage)", [f"aliasByNode({P}.CM_MON.*.TEMP3_C, 4)"], 12, 12, unit="celsius"),
]
y[0] += 8
r["panels"] += [
    ts("LGA80D STATUS_WORD", [f"aliasByNode({P}.CM_MON.*.STATUS_WORD, 4)"], 0, 12, unit="hex0x", stepped=True,
       desc="0x0840 = output off (OFF + PGOOD#); 0xFFFF = no data (NO_DATA bit)."),
    ts("Bad-temperature counters (since boot)", [f"aliasByNode({P}.CM_DIAG_LGA80D.LGA80D_BADT{{1,3}}_{{FFFF,OTHER}}.VALUE, 4)"],
       12, 12, stepped=True),
]
y[0] += 8
panels.append(r)

# ---- diagnostics: stacks ----
r = row("Diagnostics: task stacks", True, [])
r["panels"] += [
    ts("Free stack per task (words)", [f"aliasByNode(removeAboveValue({P}.CM_DIAG_STACK.*.FREE_WORDS, 65534), 4)"],
       0, 24, stepped=True, desc="High-water mark: only goes down within a boot. 65535 (task not created) is hidden."),
]
y[0] += 8
panels.append(r)

dash = {
    "title": "Apollo CM diagnostics ($WhichApollo)",
    "uid": "apollo-cm-diag",
    "tags": ["apollo", "cm_mcu"],
    "timezone": "browser",
    "schemaVersion": 38,
    "version": 1,
    "editable": True,
    "refresh": "1m",
    "time": {"from": "now-24h", "to": "now"},
    "templating": {"list": [
        {"name": "DS", "label": "Graphite", "type": "datasource", "query": "graphite", "current": {}, "hide": 0},
        {"name": "WhichApollo", "label": "Apollo", "type": "query", "datasource": DS, "query": "shelf.00.*",
         "current": {"text": "AP3010", "value": "AP3010"}, "refresh": 1, "hide": 0},
    ]},
    "links": [{"title": "Fleet overview", "type": "link", "url": "/d/apollo-cm-fleet",
               "keepTime": True, "icon": "dashboard"}],
    "annotations": {"list": []},
    "panels": panels,
}
json.dump(dash, open(sys.argv[1], "w"), indent=2)
