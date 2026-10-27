# Grafana dashboards for the CM diagnostics block

Two dashboards for the ZynqMon diagnostics block (see the cm_mcu README, "ZynqMon diagnostics
block", for what each entry means and which ones to alarm on). Both read the Graphite feed that
BUTool on the Zynq produces, with metric paths `shelf.00.<blade>.<TABLE>.<ROW>.<COLUMN>`.

| Dashboard | uid | Generator |
| --------- | --- | --------- |
| `grafana_cm_diag.json`: one blade (variable `WhichApollo`), indicators on top, diagnostics in collapsed rows | `apollo-cm-diag` | `grafana_cm_diag_make.py` |
| `grafana_cm_fleet.json`: one tile per blade per indicator, linking to the blade dashboard | `apollo-cm-fleet` | `grafana_cm_fleet_make.py` |

Edit the generator, not the JSON, and regenerate:

```bash
python grafana_cm_diag_make.py grafana_cm_diag.json
python grafana_cm_fleet_make.py grafana_cm_fleet.json
```

Import with Dashboards -> New -> Import, picking the Graphite datasource; use "Import (Overwrite)"
to replace an existing copy. The expected DIAG_SCHEMA (8) is set in both generators.

Graphite/BUTool notes: BUTool upper-cases row and column names; never use `Format=x` in the XML
for values Grafana reads (BUTool sends the text "0x..." and Graphite drops it, use the Grafana unit
`hex0x` instead); states, codes and masks need `consolidateBy(.., 'last')`, or Graphite averages
them when it merges points.
