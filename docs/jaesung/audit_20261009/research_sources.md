# 조사 출처 요약 (2026-10-09)

웹 조사 에이전트 3개의 보고를 정리했습니다.
- 수치는 원문에서 확인된 것과 확인되지 않은 것을 구분했습니다. [미확인]은 기억이나 2차 출처입니다.
- 업체 주장은 [업체], 동료심사 논문은 [PR], 프리프린트는 [PRE]로 표시했습니다.

## A. 온라인 3D 적재 (물류)

**학습 기반**
- **Zhao et al., "Online 3D Bin Packing with Constrained Deep RL"**, AAAI 2021 [PR]
  - 링크: https://arxiv.org/abs/2006.14978 · 코드 https://github.com/alexfrom0815/Online-3D-BPP-DRL
  - 안정성 규칙: 60%+4코너 / 80%+3코너 / 95%
  - 성능: RS 50.5% (휴리스틱 BPH 35.4%), 실로봇 66.3%, 사람 52.1%
  - 속도: < 10 ms
- **Zhao, Yu, Xu, "Packing Configuration Tree (PCT)"**, ICLR 2022 [PR]; 확장 arXiv 2504.04421 [PRE]
  - 코드: https://github.com/alexfrom0815/Online-3D-BPP-PCT (MIT)
  - 안정성 포함: PCT 75.8~76.0%, DBL 60.5%, HM 57.6%, CDRL 70.9%
  - 하중 제약 강도를 높이면 69.6 → 61.9 → 57.2%로 떨어짐
  - 실제 팔레타이징 로봇(ABB, 363팔레트, 6,891박스): 57.4%, 사이클 9.8 s
  - 속도: 10~40 ms
- **Xiong et al., GOPT**, IEEE RA-L 2024 [PR]
  - 링크: https://arxiv.org/abs/2409.05344 · 코드 https://github.com/Xiong5Heng/GOPT
  - 성능: 76.1%, 실로봇 67.5%
- **Gao et al., "Online 3D Bin Packing with Fast Stability Validation and Stable Rearrangement Planning"**, arXiv 2507.09123 (2025) [PRE]
  - **LBCP의 원 출처**입니다. Gzara 2020이 아닙니다.
  - 성능: 73.6% → 재배치 80.5%, 실로봇 71.2%
- **Puche & Lee, "Online 3D BPP RL with Buffer"**, IROS 2022 [PR]
  - 링크: https://arxiv.org/abs/2208.07123
  - RS 기준 버퍼 1 → 3칸에서 53.1 → 62.1%
  - rollout이 가치망보다 정확함
- **MPC/MCTS lookahead**, arXiv 2601.02649 (2026) [PRE]
  - 실제 택배 데이터에서 분포 변화 강건성 향상, 결정 약 10 s
- **Ali, Ramos, Oliveira, "Static stability versus packing efficiency in online 3D packing"**, C&OR 2025 [PR]
  - 다각형 기반 안정성 규칙이 바닥면 비율 규칙보다 빈 수가 적음 (원문 수치 미확인)
- **Ali et al., "Heuristics for online 3D packing ... algorithm selection"**, Applied Soft Computing 2024 [PR]

**휴리스틱·팔레트 적재**
- **Crainic, Perboli, Tadei, "Extreme Point-based heuristics"**, INFORMS JoC 2008 [PR]
- **Gzara, Elhedhli, Yildiz, "The Pallet Loading Problem: 3D bin packing with practical constraints"**, EJOR 287(3) 2020 [PR]
  - 층 기반 열 생성, 수직 지지 SOCP, 하중 흐름 그래프
  - 링크: https://ideas.repec.org/a/eee/ejores/v287y2020i3p1062-1074.html
- **Elhedhli, Gzara, Yildiz, "3D bin packing and mixed-case palletization"**, INFORMS J. Optimization 2019 [PR]

**업계**
- Symbotic: "수작업보다 30~50% 촘촘", 셀당 1,300 cph 이상 [업체]
- Dexterity: 무작위 순서 혼합 팔레트 99% 이상 안정 [업체]
- Mujin: 버퍼·재정렬 테이블 사용 [업체]
- **혼합 팔레트 체적 채움률을 공개한 업체는 없음.** 55~70%는 PCT 로봇 실측과 GOPT·LBCP 실로봇 결과에서 추정한 앵커입니다.

## B. 다른 분야

**테트리스**
- **Thiery & Scherrer, "Building Controllers for Tetris"**, ICGA J. 32(1) 2009 [PR]
  - BCTS: 원판 게임에서 91만 줄 이상, 2008 RL Competition 우승
- **Szita & Lőrincz, "Learning Tetris using the noisy cross-entropy method"**, Neural Computation 2006 [PR]
  - 링크: https://arxiv.org/abs/cs/0610170
- **Gabillon, Ghavamzadeh, Scherrer, "ADP Finally Performs Well in the Game of Tetris"**, NIPS 2013 [PR]
- **Algorta & Şimşek, "The Game of Tetris in Machine Learning"**, arXiv 1905.01652 (survey)

**칩 배치**
- **Mirhoseini et al.**, Nature 2021 [PR]
- **Cheng, Kahng et al., "Assessment of RL for Macro Placement"**, ISPD 2023 [PR]
  - 링크: https://arxiv.org/abs/2302.11014
  - 튜닝된 SA·상용 툴이 RL과 대등하거나 우세. Google은 반박 중이며 **논쟁 중**입니다.
- **Markov, "Reevaluating Google's RL for IC Macro Placement"**, CACM

**신경 조합최적화**
- **Kool et al.**, ICLR 2019 [PR]
- **POMO**, NeurIPS 2020 [PR]
- **NeuroLKH**, NeurIPS 2021 [PR]
- **Xia et al.**, ICML 2024 position [PR]: 학습 heatmap이 단순 heatmap보다 낫지 않음

**확률적 온라인 최적화**
- **Van Hentenryck & Bent, "Online Stochastic Combinatorial Optimization"**, MIT Press 2006
- **Bent & Van Hentenryck, "Regrets Only!"**, AAAI 2004 [PR]
- **Bertsekas, Tsitsiklis, Wu, "Rollout Algorithms for Combinatorial Optimization"**, J. Heuristics 1997 [PR]
  - rollout은 기본 휴리스틱보다 나빠지지 않음 (조건부)
- **Laterre et al., "Ranked Reward"**, arXiv 1807.01672 [PRE]
- **Danihelka et al., Gumbel MuZero**, ICLR 2022 [PR]

**동적 적치·항공화물**
- DynStack competition (GECCO)
- **Paquay et al.**, EJOR 2018 (ULD, extreme point) [PR]
- **Wang & Hauser**, ICRA 2019 (높이맵 최소화 + 안정성) [PR]

**안정성 평형·학습 대리모델**
- **Whiting et al.**, ACM TOG 2009 (LP 정역학 평형) [PR]
- **Lerer et al.**, ICML 2016 (블록탑 안정성 학습) [PR]

## C. 안정성 기준·사업성

**안정성 기준**
- **Ramos et al., "Container loading with static mechanical equilibrium"**, Transp. Res. B 91 (2016) [PR]
  - 전면 지지는 과보수적이고, 정역학 평형은 효율과 안정 보장을 함께 얻음
- **Mazur, Gamer, Ramos, Schoder, "Standing on a common ground"**, ITOR 2025 [PR, 오픈액세스]
  - 링크: https://kups.ub.uni-koeln.de/80554/
  - 다물체 시뮬레이션 대비 비교: 전면 지지가 가장 제한적이고, 모든 근사법에 구조적 오류가 있음
- **UPS 특허 US6699007** (만료): 무게에 따라 지지 요구가 50%(≤30 lb)에서 65%(70 lb)로 증가
- **McKee 식**: 개별 상자 예측 오차가 큼 (R² 0.737, PMC8124728)
  - 환경 계수 [업계 경험칙]: 습도 70%에서 0.8, 90일 적재 시 0.55
- **EUMOS 40509**: 0.5 g 가속 시험 [시험기관]
- **EU 지침 2014/47/EU**: 전방 0.8 g / 측면 0.5 g
- **무게 역전 금지 비율에 대한 업계 표준은 찾지 못함.** 연구·업계는 압괴·하중 모델로 처리합니다. 다만 공식 미션 문구가 요구하므로 우리는 유지합니다.

**사업성 [업체/시장보고서]**
- 혼합 팔레타이징 시장: 2025년 약 USD 3.2B, CAGR 7.8~9.4%
- 셀 비용: 협동로봇 USD 50~120k, 산업용 USD 150~400k
- 회수 기간: 10~18개월(협동로봇) / 2~4년(산업용)
- 처리량: 수작업 300~450 cph, 로봇 혼합 600~1,000 cph, Symbotic 1,300 cph
- 국내 택배: 2025년 64.2억 건 (etnews 2026-04-24 보도)
- 2026 최저임금: 10,320원/h
- **적재율과 물류비**: 영국 FMCG 사례에서 출하 −9.09% → 5개월 £207k 절감 (MATEC 2024)
- **국내 팔레트당 운임 공개 자료는 없음** → 운송사 견적 필요

**IP**
- Mujin: 높이맵·배치 점수·재계획 특허군 (US10696493 외)
- Symbotic: 유연 순서 팔레트 구성 (US11305430 외)
- 출원 전 FTO 조사가 필요합니다.
