export const scenarios = {
  "offset-bay": {
    label: "OFFSET APPROACH",
    description: "从车位右前方横向接近，再转向倒入两车之间的中央车位。",
    ui: { zh: { label: "右侧错位接近", description: "从车位右前方横向接近，再转向倒入两车之间的中央车位。" }, en: { label: "Offset approach", description: "Approach laterally from the front-right, then turn and reverse into the center bay." } },
    task: "Approach from the right side of the aisle and park in the center bay between two parked vehicles.",
    start: { x: 2.65, y: 0.25, heading: 180 },
    target: { x: 0, y: -2.45, heading: 90 },
  },
  "tight-corner": {
    label: "TIGHT ANGLED APPROACH",
    description: "从右侧斜向进入狭窄通道，需要前进与倒车调整后入库。",
    ui: { zh: { label: "狭角调整入库", description: "从右侧斜向进入狭窄通道，需要前进与倒车调整后入库。" }, en: { label: "Tight angled approach", description: "Enter from a tight angle near the right boundary, then reposition before parking." } },
    task: "Recover from an angled approach near the right boundary, then park in the center bay between two vehicles.",
    start: { x: 4.05, y: 0.65, heading: -145 },
    target: { x: 0, y: -2.45, heading: 90 },
  },
  "reverse-entry": {
    label: "STRAIGHT REVERSE ENTRY",
    description: "车辆已基本对准目标车位，主要考察直线倒车与最后的微调。",
    ui: { zh: { label: "直线倒车入库", description: "车辆已基本对准目标车位，主要考察直线倒车与最后的微调。" }, en: { label: "Straight reverse entry", description: "The car is almost aligned with the bay, testing straight reverse motion and final corrections." } },
    task: "The vehicle is nearly aligned with the bay; reverse into it and make small corrections.",
    start: { x: 0.22, y: 0.35, heading: 88 },
    target: { x: 0, y: -2.45, heading: 90 },
  },
  "left-offset": {
    label: "LEFT-SIDE APPROACH",
    description: "从车位左前方横向接近，考察与右侧来车方向相反的转向和倒车选择。",
    ui: { zh: { label: "左侧错位接近", description: "从车位左前方横向接近，考察与右侧来车方向相反的转向和倒车选择。" }, en: { label: "Left-side approach", description: "Approach laterally from the front-left, reversing the steering choices of the right-side scenario." } },
    task: "Approach from the left side of the aisle and park in the center bay between two parked vehicles.",
    start: { x: -2.65, y: 0.25, heading: 0 },
    target: { x: 0, y: -2.45, heading: 90 },
  },
  "forward-entry": {
    label: "FORWARD BAY ENTRY",
    description: "车辆位于车位正前方，车头朝向车位，考察前进入库和居中控制。",
    ui: { zh: { label: "前进入库", description: "车辆位于车位正前方，车头朝向车位，考察前进入库和居中控制。" }, en: { label: "Forward bay entry", description: "Start in front of the bay and drive forward, testing centering and alignment." } },
    task: "Drive forward into the center bay between two parked vehicles and finish centered and aligned.",
    start: { x: 0.18, y: 1.15, heading: -88 },
    target: { x: 0, y: -2.45, heading: -90 },
  },
  "wide-angle": {
    label: "WIDE ANGLE RECOVERY",
    description: "从左上方较远位置斜向接近，需要先进入通道，再建立合适的入库姿态。",
    ui: { zh: { label: "大角度恢复", description: "从左上方较远位置斜向接近，需要先进入通道，再建立合适的入库姿态。" }, en: { label: "Wide angle recovery", description: "Approach diagonally from farther left, enter the aisle, and establish a viable parking pose." } },
    task: "Recover from a wide angled start, follow the open aisle, and park in the center bay between two vehicles.",
    start: { x: -4.0, y: 2.15, heading: 24 },
    target: { x: 0, y: -2.45, heading: 90 },
  },
  custom: {
    label: "CUSTOM START",
    description: "拖动车身改变起点；拖动车头前方的蓝色方向手柄改变角度。两边模型使用同一姿态。",
    ui: { zh: { label: "自定义起点…", description: "拖动车身改变起点；拖动车头前方的蓝色方向手柄改变角度。两边模型使用同一姿态。" }, en: { label: "Custom start…", description: "Drag the car to change its starting point, or drag the blue handle to rotate it. Both models use the same pose." } },
    task: "Park the vehicle from the user-defined starting pose in the center bay between two parked vehicles.",
    start: { x: 2.65, y: 0.25, heading: 180 },
    target: { x: 0, y: -2.45, heading: 90 },
    editable: true,
  },
};

export const scenarioTasks = Object.fromEntries(
  Object.entries(scenarios).map(([id, scenario]) => [id, scenario.task]),
);
