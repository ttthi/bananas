## 📄 JSON Format

```json
{
  "pallet_size": [width_cm, length_cm, height_cm],
  "configurations": [
    [
      ["block_type", [x_center_cm, y_center_cm, z_center_cm]],
      ...
    ]
  ]
}
```

---

## 🧠 Field Explanations

| Field | Description |
|:---|:---|
| `pallet_size` | A list `[width, length, height]` in centimeters (cm) specifying the size of the pallet. |
| `configurations` | A list containing one or more stacking configurations. Each configuration is a list of blocks. |
| Inside each configuration | Each block is represented by a list: `["block_type", [x_center, y_center, z_center]]`. It includes the block type (e.g., `"cube, 3.8cm"`) and its center position in 3D space. |

---

## 📐 Block Details

| Property | Value |
|:---|:---|
| Block base size | 3.8 cm |
| Margin between blocks (X and Y dimensions) | 0.3 cm |
| Effective block size | 4.1 cm (3.8 cm + 0.3 cm) |
| Block positioning | Center coordinates `[x, y, z]` are specified relative to the pallet's bottom-left-back corner at (0,0,0). |

- **Width (X-axis)**: left → right
- **Length (Y-axis)**: back → front
- **Height (Z-axis)**: bottom → top
- Blocks are placed **centered** at the `[x, y, z]` position.
- **Margin of 0.3 cm** is included **in X and Y directions** to prevent blocks from touching each other.

---
