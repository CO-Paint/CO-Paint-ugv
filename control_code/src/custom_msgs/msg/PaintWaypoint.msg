# PaintWaypoint.msg
# 도색 경로의 단일 웨이포인트
#
# paint_on  : False면 해당 구간 분사 중단 (금지구역 통과 시)
# speed     : 해당 구간 비행 속도 제한 (0.0 = 기본값 사용)

float32 x
float32 y
float32 z
bool    paint_on
float32 speed
