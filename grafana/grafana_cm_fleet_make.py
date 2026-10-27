# Generates grafana_cm_fleet.json: one tile per blade for each diagnostics indicator.
# Diagnostics for a blade are on the per-blade dashboard (grafana_cm_diag_make.py); every
# tile links there. Usage: python grafana_cm_fleet_make.py grafana_cm_fleet.json
import json, sys

ALL = "shelf.00.*"            # one series per blade; node 2 is the blade name
DIAG_SCHEMA = 8               # layout the per-blade dashboard was built for
BLADE_UID = "apollo-cm-diag"  # uid of the per-blade dashboard
DS = {"type": "graphite", "uid": "${DS}"}
pid = [0]
y = [0]

def nid():
    pid[0] += 1
    return pid[0]

def thresholds(steps):
    return {"mode": "absolute", "steps": [{"color": c, "value": v} for v, c in steps]}

def mapping(d):
    return [{"type": "value", "options": {str(k): {"text": v, "index": i} for i, (k, v) in enumerate(d.items())}}]

LINK = [{"title": "Open ${__series.name}", "targetBlank": False,
         "url": f"/d/{BLADE_UID}?var-WhichApollo=${{__series.name}}&${{__url_time_range}}"}]

def tiles(title, expr, steps, desc, calc="lastNotNull", maps=None, unit="none", h=4):
    p = {"id": nid(), "type": "stat", "title": title, "description": desc, "datasource": DS,
         "gridPos": {"x": 0, "y": y[0], "w": 24, "h": h},
         "targets": [{"refId": "A", "target": expr, "datasource": DS}],
         "options": {"reduceOptions": {"calcs": [calc], "fields": "", "values": False},
                     "colorMode": "background", "graphMode": "none", "textMode": "value_and_name",
                     "orientation": "vertical", "justifyMode": "auto"},
         "fieldConfig": {"defaults": {"unit": unit, "thresholds": thresholds(steps),
                                      "mappings": mapping(maps) if maps else [],
                                      "color": {"mode": "thresholds"}, "links": LINK},
                         "overrides": []}}
    y[0] += h
    return p

def per_blade(path, func):  # one series per blade, merged points keep `func`
    return f"aliasByNode(consolidateBy({ALL}.{path}, '{func}'), 2)"

def increase_per_blade(path):  # increase over the range per blade, counter resets ignored
    # consolidateBy renames the series, so aliasByNode(.., 0) restores the blade name
    return (f"aliasByNode(consolidateBy(groupByNode(nonNegativeDerivative(keepLastValue({ALL}.{path})), 2, 'sum'), 'sum'), 0)")

PWR = {0: "FAILURE", 1: "INIT", 2: "DOWN", 3: "OFF", 4: "L1ON", 5: "L2ON", 6: "L3ON", 7: "L4ON",
       8: "L5ON", 9: "L6ON", 10: "ON", 11: "CONFIG"}
FATAL = {0: "none", 7: "HARD FAULT", 8: "ASSERT", 9: "STACK OVERFLOW", 21: "MALLOC FAIL"}

panels = [
    tiles("Power state", per_blade("CM_DIAG_PWR.PWR_STATE.VALUE", "last"),
          ((None, "red"), (1, "blue"), (10, "green"), (11, "blue")),
          "POWER_FAILURE red, ON green, anything else (OFF, sequencing) blue.", maps=PWR),
    tiles("MCU health", per_blade("CM_DIAG.MCU_HEALTH.VALUE", "max"), ((None, "green"), (1, "red")),
          "Non-zero: power fault, temperature or voltage alarm active now.", maps={0: "OK"}),
    tiles("Previous boot ended in", per_blade("CM_DIAG_RESET.PREV_FATAL_CODE.VALUE", "last"),
          ((None, "green"), (1, "red")), "Fatal error-log entry of the previous boot.", maps=FATAL),
    tiles("Uptime", per_blade("CM_MON.MCU_UPTIME.MINUTES", "last"),
          ((None, "orange"), (60, "green")), "Current MCU uptime; orange below 60 min.", unit="m"),
    # a reboot is a drop in MCU_UPTIME: derivative < 0, isNonNull turns each drop into a 1
    tiles("Reboots in range",
          f"aliasByNode(consolidateBy(groupByNode(isNonNull(removeAboveValue(derivative(keepLastValue("
          f"{ALL}.CM_MON.MCU_UPTIME.MINUTES)), -1)), 2, 'sum'), 'sum'), 0)",
          ((None, "green"), (1, "orange")), "MCU resets in the time range (drops of MCU_UPTIME).",
          calc="sum"),
    tiles("Power failures in range",
          increase_per_blade("CM_DIAG_PWR.{PWR_SUPPLY_FAIL,PWR_CONFIG_FAIL}.VALUE"),
          ((None, "green"), (1, "red")), "Supply failures and LGA80D_init failures.", calc="sum"),
    tiles("Alarm power-downs in range", increase_per_blade("CM_DIAG_PWR.PWR_ALARM_OFF.VALUE"),
          ((None, "green"), (1, "orange")), "Temperature/voltage alarm power-downs (protection worked).",
          calc="sum"),
    tiles("I2C master errors in range", increase_per_blade("CM_DIAG_I2C.*.{START_FAIL,ERROR}"),
          ((None, "green"), (1, "orange"), (100, "red")), "All 5 buses; NACKs are not counted.", calc="sum"),
    tiles("I2C slave (IPMC) events in range",
          increase_per_blade("CM_DIAG_I2CS.{I2CS_DUP_IRQ,I2CS_UNHANDLED}.VALUE"),
          ((None, "green"), (1, "red")), "Duplicate DATA interrupts and unhandled I2CSCSR values.", calc="sum"),
    tiles("LGA80D bad temperature reads in range",
          increase_per_blade("CM_DIAG_LGA80D.{LGA80D_BADT1_FFFF,LGA80D_BADT1_OTHER,LGA80D_BADT3_FFFF,LGA80D_BADT3_OTHER}.VALUE"),
          ((None, "green"), (1, "orange")), "READ_TEMPERATURE_1/_3 words outside 0..175 C.", calc="sum"),
    tiles("Min free stack (words)",
          f"aliasByNode(consolidateBy(groupByNode(removeAboveValue({ALL}.CM_DIAG_STACK.*.FREE_WORDS, 65534), 2, 'min'), 'min'), 0)",
          ((None, "red"), (32, "orange"), (64, "green")), "Lowest high-water mark over all tasks.", calc="min"),
    tiles("Diag schema", per_blade("CM_DIAG.DIAG_SCHEMA.VALUE", "last"),
          ((None, "red"), (DIAG_SCHEMA, "green"), (DIAG_SCHEMA + 1, "red")),
          f"Green only at {DIAG_SCHEMA}, the layout these dashboards expect: other values mean older "
          "or newer firmware/XML."),
]

def ts(title, expr, unit, desc, h=8, bars=False):
    custom = {"drawStyle": "bars" if bars else "line", "fillOpacity": 80 if bars else 0,
              "lineInterpolation": "linear", "spanNulls": not bars, "showPoints": "never"}
    p = {"id": nid(), "type": "timeseries", "title": title, "description": desc, "datasource": DS,
         "gridPos": {"x": 0, "y": y[0], "w": 24, "h": h},
         "targets": [{"refId": "A", "target": expr, "datasource": DS}],
         "options": {"legend": {"displayMode": "table", "placement": "right", "showLegend": True,
                                "calcs": ["sum"] if bars else ["lastNotNull"]},
                     "tooltip": {"mode": "multi", "sort": "desc"}},
         "fieldConfig": {"defaults": {"unit": unit, "custom": custom, "color": {"mode": "palette-classic"},
                                      "links": LINK}, "overrides": []}}
    y[0] += h
    return p

panels += [
    ts("LGA80D bad temperature reads per hour",
       "groupByNode(summarize(nonNegativeDerivative(keepLastValue(" + ALL +
       ".CM_DIAG_LGA80D.{LGA80D_BADT1_FFFF,LGA80D_BADT1_OTHER,LGA80D_BADT3_FFFF,LGA80D_BADT3_OTHER}.VALUE)), '1h', 'sum'), 2, 'sum')",
       "none", "Fleet-wide rate of the -0.50 C (0xFFFF) and other bad LGA80D temperature words.", bars=True),
    ts("MCU uptime", per_blade("CM_MON.MCU_UPTIME.MINUTES", "min"), "m",
       "A drop to 0 is a reset; drops on several blades at once point to a shelf event."),
]

dash = {
    "title": "Apollo CM fleet",
    "uid": "apollo-cm-fleet",
    "tags": ["apollo", "cm_mcu"],
    "timezone": "browser",
    "schemaVersion": 38,
    "version": 1,
    "editable": True,
    "refresh": "1m",
    "time": {"from": "now-24h", "to": "now"},
    "links": [{"title": "Blade diagnostics", "type": "link", "url": f"/d/{BLADE_UID}",
               "keepTime": True, "icon": "dashboard"}],
    "templating": {"list": [
        {"name": "DS", "label": "Graphite", "type": "datasource", "query": "graphite", "current": {}, "hide": 0}]},
    "annotations": {"list": []},
    "panels": panels,
}
json.dump(dash, open(sys.argv[1], "w"), indent=2)
