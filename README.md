# Heatro Viewer

조선 외판 핫밴딩(Hot Bending) 레이저 스캔 + 열 프로파일 JSON을 브라우저에서 3D로 시각화하는 뷰어.
Three.js 기반 **정적 페이지** (서버/빌드 불필요). 데이터는 **로컬에서만** 처리되며 외부로 전송되지 않음.

## 🔗 Live Viewer

GitHub Pages 로 배포됨 → [https://chihun-lee.github.io/heatro/viewer/](https://chihun-lee.github.io/heatro/viewer/)

업로드 패널에 `.json` 파일을 드래그하거나 "파일 선택"으로 불러오면 됨.

## 데이터 스키마 (요약)

Heat-Ro 장비(NACKS_01 등)에서 내보내는 단일 JSON 파일 한 개가 = **한 번의 작업 사이클 스냅샷**.

| 필드 | 의미 |
|---|---|
| `SourceGrid.Points` | 가열 **전** 판재 점군 (행×열 2D 격자, 3D 좌표 문자열) |
| `TargetGrid.Points` | 설계 목표 형상 점군 |
| `designBouEdges` / `measureBouEdges` | 설계/측정 외곽 폴리라인 |
| `BendingCurves[]` | 계획된 가열선 궤적 + 모든 열 파라미터 |
| `complatedRatio1/2`, `ResidualNorm` | 완성도 %, 잔차 norm |
| `seamNames`, `directionNames` | Seam·방향 라벨 (AFT/FORE/TOP/BOTTOM 등) |

**가열 후 스캔**은 같은 JSON 안에 없고, **다음 iteration의 `SourceGrid`** 로 나타남. 따라서 before/after 비교는 뷰어의 **Compare** 기능으로 여러 JSON을 함께 올려서 확인.

## 주요 기능

- 🔼 **로컬 업로드**: 드래그앤드롭 · `.json` 다중 선택. 데이터가 서버로 전송되지 않음.
- 🧩 **Multi-Dataset**: 여러 파일을 동시에 적재하고, Primary(상세) + Compare(Source 표면만 겹치기) 로 선택.
- 🎚 **Layer on/off + 투명도**: Source/Target/Deviation/경계선/가열선/입력점/라벨/그리드/좌표축 각 개별. "모두 켜기 / 모두 끄기" 버튼.
- 🔥 **Bending Curve 개별 토글** + 클릭/호버로 파라미터(온도, 속도, 냉각시간, Shape, Step, Repeat 등) 상세 표시.
- 🌈 **Deviation Heatmap**: Source↔Target 격자 거리(mm)를 viridis 컬러맵으로 Source 표면에 매핑, 범례 자동 계산.
- 🧭 카메라: Orbit + Top/Side 프리셋, 전체 프레이밍, Wireframe 토글.

## 로컬 실행

정적 파일이라 아무 HTTP 서버면 된다.

```bash
cd viewer
python3 -m http.server 8765
# http://localhost:8765/
```

## 로컬 개발 시 데이터 전처리 (선택)

개발 중 실제 JSON 로드 없이 구조만 확인하려면 `viewer/build_data.py` 로 샘플을 `data.js` 로 구워 옛 방식으로 테스트할 수 있음. 배포 파이프라인에는 들어가지 않음 (`.gitignore` 로 제외).

## 라이선스 / 주의

- 데이터(원본 JSON, PPT, PDF)는 **저장소에 커밋되지 않음** (`.gitignore`).
- 뷰어 코드 자체는 연구용 오픈.
