"""원본 Heatro JSON을 뷰어용 data.js 로 변환.

- Point 문자열("x, y, z") → [x, y, z] 숫자 배열로 파싱
- SourceGrid / TargetGrid 2D 격자 유지
- designBouEdges / measureBouEdges 폴리라인 유지
- BendingCurves 는 궤적 Points + 모든 파라미터 유지
"""
from __future__ import annotations

import json
import os
import sys

DATA_DIR = os.path.join(os.path.dirname(__file__), "..", "data", "Heatro_DATA")
OUT_PATH = os.path.join(os.path.dirname(__file__), "data.js")


def parse_point(s):
    if s is None:
        return None
    if isinstance(s, (list, tuple)):
        return [float(x) for x in s]
    parts = [p.strip() for p in str(s).split(",")]
    return [float(p) for p in parts[:3]]


def parse_points_1d(seq):
    return [parse_point(p) for p in seq] if seq else []


def parse_points_2d(grid):
    return [parse_points_1d(row) for row in grid] if grid else []


def parse_name_points(arr):
    return [
        {"point": parse_point(item.get("textPoint")), "text": item.get("text", "")}
        for item in (arr or [])
        if item.get("textPoint")
    ]


def convert(doc):
    src = doc.get("SourceGrid", {}) or {}
    tgt = doc.get("TargetGrid", {}) or {}

    curves = []
    for c in doc.get("BendingCurves", []) or []:
        curves.append({
            "curveId": c.get("CurveId"),
            "heatingMethod": c.get("HeatingMethod"),
            "points": parse_points_1d(c.get("Points")),
            "inputPoints": parse_points_1d(c.get("inputUserPointList")),
            "strength": c.get("Strength"),
            "width": c.get("Width"),
            "heatIntensity": c.get("HeatIntensity"),
            "split": c.get("Split"),
            "weavingRepeat": c.get("WeavingRepeat"),
            "thk": c.get("THK"),
            "speed": c.get("speed"),
            "coolingTime": c.get("coolingTime"),
            "paramShape": c.get("ParameterShape"),
            "paramTemp": c.get("ParameterTemp"),
            "paramStep": c.get("ParameterStep"),
            "paramRepeat": c.get("ParameterRepeat"),
            "paramCoolingSpeed": c.get("ParameterCoolingSpeed"),
            "isAutoMaked": c.get("isAutoMaked"),
        })

    return {
        "legacyId": doc.get("LegacyId"),
        "partName": doc.get("sPartName"),
        "plateName": doc.get("sPlateName"),
        "curveType": doc.get("CurveType"),
        "thickness": doc.get("dThickNess"),
        "deviceCode": doc.get("DeviceCode"),
        "createdAt": doc.get("CreateDateTime"),
        "directionKey": doc.get("DirectionKey"),
        "completedRatio1": doc.get("complatedRatio1"),
        "completedRatio2": doc.get("complatedRatio2"),
        "residualNorm": doc.get("ResidualNorm"),
        "totalCurveLength": doc.get("TotalCurveLength"),
        "curveCount": doc.get("CurveCount"),
        "designMirror": doc.get("designMirror"),
        "designFlip": doc.get("designFlip"),
        "designRotate": doc.get("designRotate"),
        "designRotate90": doc.get("designRotate90"),
        "sourceGrid": {
            "resolution": src.get("Resolution"),
            "material": src.get("Material"),
            "thickness": src.get("Thickness"),
            "centerX": src.get("CenterX"),
            "centerY": src.get("CenterY"),
            "directionRad": src.get("DirectionRad"),
            "points": parse_points_2d(src.get("Points")),
        },
        "targetGrid": {
            "resolution": tgt.get("Resolution"),
            "material": tgt.get("Material"),
            "thickness": tgt.get("Thickness"),
            "centerX": tgt.get("CenterX"),
            "centerY": tgt.get("CenterY"),
            "directionRad": tgt.get("DirectionRad"),
            "points": parse_points_2d(tgt.get("Points")),
        },
        "designBouEdges": [parse_points_1d(e) for e in (doc.get("designBouEdges") or [])],
        "measureBouEdges": [parse_points_1d(e) for e in (doc.get("measureBouEdges") or [])],
        "seamNames": parse_name_points(doc.get("seamNames")),
        "directionNames": parse_name_points(doc.get("directionNames")),
        "bendingCurves": curves,
    }


def main():
    files = sorted(f for f in os.listdir(DATA_DIR) if f.endswith(".json"))
    datasets = []
    for f in files:
        path = os.path.join(DATA_DIR, f)
        with open(path, encoding="utf-8-sig") as fp:
            doc = json.load(fp)
        conv = convert(doc)
        conv["fileName"] = f
        datasets.append(conv)

    payload = json.dumps(datasets, ensure_ascii=False, separators=(",", ":"))
    with open(OUT_PATH, "w", encoding="utf-8") as fp:
        fp.write("window.HEATRO_DATA = ")
        fp.write(payload)
        fp.write(";\n")

    total_bytes = os.path.getsize(OUT_PATH)
    print(f"wrote {OUT_PATH} ({total_bytes/1024:.1f} KB, {len(datasets)} datasets)")
    for d in datasets:
        print(
            f"  {d['fileName']}: {d['partName']} curves={d['curveCount']} "
            f"ratio1={d['completedRatio1']:.2f} residual={d['residualNorm']:.2f}"
        )


if __name__ == "__main__":
    main()
