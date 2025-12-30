# HLK-LD2402-L 配置示例

## 基本配置要求

所有使用 `hlk_ld2402l` 平台的传感器都必须包含 `hlk_ld2402l_id` 字段，该字段引用主组件的 ID。

## 正确配置示例

```yaml
# 1. 首先定义主组件
hlk_ld2402l:
  uart_id: uart_bus
  id: radar_sensor  # 这是主组件的 ID
  max_distance: 6.3
  timeout: 5

# 2. 配置二进制传感器（必须包含 hlk_ld2402l_id）
binary_sensor:
  - platform: hlk_ld2402l
    id: radar_presence
    name: "Presence"
    device_class: presence
    hlk_ld2402l_id: radar_sensor  # ⚠️ 必需字段：引用主组件 ID

  - platform: hlk_ld2402l
    id: radar_micromovement
    name: "Micromovement"
    device_class: motion
    hlk_ld2402l_id: radar_sensor  # ⚠️ 必需字段

# 3. 配置传感器（必须包含 hlk_ld2402l_id）
sensor:
  - platform: hlk_ld2402l
    id: radar_distance
    name: "Distance"
    hlk_ld2402l_id: radar_sensor  # ⚠️ 必需字段
    device_class: distance
    unit_of_measurement: "cm"

  - platform: hlk_ld2402l
    id: radar_light
    name: "Light"
    hlk_ld2402l_id: radar_sensor  # ⚠️ 必需字段
    device_class: illuminance
    unit_of_measurement: "lux"
    light: true

# 4. 配置文本传感器（必须包含 hlk_ld2402l_id）
text_sensor:
  - platform: hlk_ld2402l
    id: radar_firmware_version
    name: "Radar Firmware Version"
    hlk_ld2402l_id: radar_sensor  # ⚠️ 必需字段
    firmware_version: true
```

## 常见错误

### ❌ 错误配置（缺少 hlk_ld2402l_id）

```yaml
binary_sensor:
  - platform: hlk_ld2402l
    id: radar_presence
    name: "Presence"
    device_class: presence
    # ❌ 缺少 hlk_ld2402l_id 字段
```

### ✅ 正确配置

```yaml
binary_sensor:
  - platform: hlk_ld2402l
    id: radar_presence
    name: "Presence"
    device_class: presence
    hlk_ld2402l_id: radar_sensor  # ✅ 必需字段
```

## 注意事项

1. **主组件 ID**：在 `hlk_ld2402l:` 部分定义的 `id` 必须与所有子传感器中的 `hlk_ld2402l_id` 匹配
2. **所有传感器都需要**：无论是 `binary_sensor`、`sensor` 还是 `text_sensor`，只要使用 `platform: hlk_ld2402l`，就必须包含 `hlk_ld2402l_id` 字段
3. **ID 命名**：主组件的 `id` 可以是任何名称（如 `radar_sensor`、`ld2402` 等），但所有子传感器必须引用相同的 ID

## 完整配置示例

请参考项目中的 `hlk_ld2402_minimal.yaml` 或 `hlk_ld2402_complete.yaml` 文件获取完整配置示例。

