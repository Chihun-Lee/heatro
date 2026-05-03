# Heatro — 조선 외판 핫밴딩 ML 프로젝트

## 프로젝트 개요
- **클라이언트**: 기득산업(KIDUCK INDUSTRY). 장비명 Heat-Ro(K061/K062/K064/K066). 한화오션, 난통 코스코, 이마바리 조선소 등에 납품.
- **최종 목표**: 설계 목표 곡면(Target) ↔ 초기 평판(Source) 스캔 데이터 + 실제 적용된 열 프로파일(BendingCurves) 의 쌍을 대량 수집해, **원하는 형상을 만드는 열 프로파일을 역설계하는 딥러닝 모델**을 학습.
- **작업 루프 (PPT 4장 참조)**: 평판 안착 → 레이저 스캔 → 설계와 비교 → 가열선 생성(자동/수동) → 4대 로봇팔이 토치로 화염 가열 + 열화상 모니터링 → 재스캔 → 완성도 85% 이상까지 반복, 95% 이상에서 완료.

## 디렉토리
```
heatro/
├── viewer/                 # 브라우저 3D 뷰어 (Three.js, 정적 페이지)
│   ├── index.html
│   ├── app.js              # 업로드 · 레이어 · compare · 메타 표시
│   ├── style.css
│   └── build_data.py       # 개발용 JSON → data.js 전처리 (옵션)
├── data/                   # 실제 JSON 샘플 (Heatro_DATA/*.json) — gitignore
├── *.pdf / *.pptx          # 기득산업 소개서 — gitignore
├── README.md               # 뷰어 사용 안내 (배포용 Pages 링크 포함)
└── CLAUDE.md               # (이 파일)
```

## 데이터 스키마 핵심
- 한 JSON = **한 번의 작업 사이클 스냅샷**. 파일명은 GUID.
- `SourceGrid.Points` (가열 전 스캔, 행×열 2D 격자, 각 점은 `"x, y, z"` 문자열 — BOM/UTF-8-sig 있음)
- `TargetGrid.Points` (설계 목표 형상, 동일 해상도)
- `designBouEdges` / `measureBouEdges` (외곽 폴리라인 5개)
- `BendingCurves[]` (가열선 배열). 각 curve 엔트리의 주요 필드:
  - 궤적: `Points`(3D 궤적), `CurveUV`(UV 파라미터), `inputUserPointList`(사용자 입력 시·종점)
  - 열 파라미터: `HeatingMethod`, `speed`, `coolingTime`, `ParameterTemp` (예: `[800, 900]` °C), `ParameterShape`, `ParameterStep`, `ParameterRepeat`, `ParameterCoolingSpeed`
  - 기타: `Strength`, `Width`, `HeatIntensity`, `Split`, `WeavingRepeat`, `THK`, `isAutoMaked`
- 평가 지표: `complatedRatio1/2`(완성도 %), `ResidualNorm`
- 방향 라벨: `directionNames` = AFT/FORE/TOP/BOTTOM, `seamNames` = S401 등.
- **가열 후 스캔은 같은 JSON에 없음**. 다음 iteration의 `SourceGrid`가 직전 가열 결과에 해당. Compare 기능으로 두 파일을 겹쳐서 본다.

## 뷰어 개발 규약
- Three.js(0.160, unpkg CDN) + 순수 ES modules. 빌드 도구/번들러 없음.
- 뷰어는 **데이터를 번들링하지 않는다** — 사용자가 로컬에서 JSON 업로드.
- 좌표 단위 **mm**. 카메라 far=50000. Z-up (CAD 관습).
- 가열선은 WebGL linewidth 제약 때문에 `Line` 대신 `TubeGeometry` (반경 8mm) 로 렌더.
- Source/Target 모두 `depthWrite=false` + `renderOrder` 로 투명 오버랩 처리.
- Deviation Heatmap은 Source 메쉬의 vertex color 를 viridis 5-anchor 로 채우고 `material.vertexColors` 토글.

## 배포
- GitHub Pages (Repo: `Chihun-Lee/heatro`). `main` 브랜치 루트 기준 `/viewer/` 에 퍼블리싱.
- 외부 링크: `https://chihun-lee.github.io/heatro/viewer/`
- 업로드된 데이터는 **브라우저 메모리 안에서만** 처리. 네트워크 전송 없음.

## 자주 하는 작업
- 샘플 JSON 로컬 테스트: `python3 -m http.server 8765` (viewer 디렉토리) → 브라우저에서 `data/Heatro_DATA/*.json` 드래그.
- 필드 확인: `python3 -c "import json; d=json.load(open('data/Heatro_DATA/xxx.json', encoding='utf-8-sig')); ..."`.

## 하지 말 것
- 원본 JSON/PDF/PPTX 커밋 금지 (`.gitignore` 적용됨).
- Public repo 의 README 나 코드에서 데이터 파일 경로를 하드코딩해서 공개하지 않기.
- 뷰어에서 사용자 데이터를 네트워크로 보내는 기능 추가 금지.
