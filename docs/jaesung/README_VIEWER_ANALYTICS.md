# PAC2026 AHEAD Viewer Analytics Patch

이 패치는 기존 PyBullet/AHEAD 백엔드 코드는 건드리지 않고
`viewer/ahead_live/` 3개 파일만 갱신합니다.

추가 기능:
- PyBullet pallet contact-force heatmap 유지
- 4분면 실제 하중 분포 bar graph (N)
- 박스별 아래로 전달되는 실제 contact force graph (N)
- 실제 pose 기반 CoM trajectory top-view
- CoP(center of pressure) 표시
- stack height / allowed-volume utilization / CoM offset 시간 그래프
- 기존 Z-up camera 수정 포함

적용:
```bash
cd ~/AHEAD/pac2026_hdr50_proxy_scaffold
unzip -o ~/다운로드/PAC2026_ahead_viewer_analytics_patch.zip -d .
```

서버가 실행 중이면 브라우저에서 Ctrl+Shift+R.
안 되면 서버를 Ctrl+C 후 다시:
```bash
python3 scripts/run_ahead_simulator.py
```

주의:
- Chart.js는 CDN에서 로드됩니다.
- 그래프 값은 Viewer가 WebSocket으로 받은 PyBullet 실제 상태/접촉값에서 계산합니다.
