# Simulation parameters (variant_b scene)

## Sensor (2D, Hokuyo URG-04LX-like)

| parameter | symbol | value |
|---|---|---|
| angular FOV | Φ | 240° |
| beams | N | 241 (Δβ = 1.0°) |
| min range | R_min | 0.02 m |
| timing jitter | σ_t | 66.7 ps |
| range noise | σ_R = cσ_t/2 | 1.00 cm |
| power noise | σ_P | 0.1 P_th |
| calibration | K/P_th | 34.84 m² (R_max = 5.6 m at ρ = 0.9) |
| specular lobe width | θ_w | 1.5° |
| specular gain | K_s/K | 730 |
| max bounces | B | 5 |
| return policy | — | strongest |

## Materials

| material | parameters |
|---|---|
| matte walls | ρ = 0.7 (dark 0.1, bright 0.9); R_max = 4.94 / 1.87 / 5.60 m |
| mirror | R_m = 0.95 |
| glass | n = 1.5 (R_F = 0.04 head-on) |

## 3D sensor (VLP-16-like geometry)

| parameter | value |
|---|---|
| rings | 16, -15° … 15° |
| azimuth step | 1.0° |
| mounting height | 0.6 m |

## Scene surfaces

| # | name | material | from | to |
|---|---|---|---|---|
| 0 | wall | matte ρ=0.7 | (0.00, 0.00) | (4.00, 0.00) |
| 1 | bright wall | matte ρ=0.9 | (4.00, 0.00) | (6.50, 0.00) |
| 2 | wall | matte ρ=0.7 | (6.50, 0.00) | (9.00, 0.00) |
| 3 | wall | matte ρ=0.7 | (9.00, 0.00) | (9.00, 2.50) |
| 4 | dark wall | matte ρ=0.1 | (9.00, 2.50) | (9.00, 4.50) |
| 5 | wall | matte ρ=0.7 | (9.00, 4.50) | (9.00, 6.00) |
| 6 | wall | matte ρ=0.7 | (9.00, 6.00) | (0.00, 6.00) |
| 7 | wall | matte ρ=0.7 | (0.00, 6.00) | (0.00, 0.00) |
| 8 | mirror | mirror | (1.89, 3.51) | (0.91, 4.49) |
| 9 | glass | glass | (5.00, 1.00) | (6.50, 3.20) |
| 10 | object | matte ρ=0.6 | (7.37, 1.32) | (7.95, 1.50) |
| 11 | object | matte ρ=0.6 | (7.95, 1.50) | (7.83, 1.88) |
| 12 | object | matte ρ=0.6 | (7.83, 1.88) | (7.25, 1.70) |
| 13 | object | matte ρ=0.6 | (7.25, 1.70) | (7.37, 1.32) |
| 14 | box | matte ρ=0.5 | (2.95, 0.75) | (3.45, 0.75) |
| 15 | box | matte ρ=0.5 | (3.45, 0.75) | (3.45, 1.25) |
| 16 | box | matte ρ=0.5 | (3.45, 1.25) | (2.95, 1.25) |
| 17 | box | matte ρ=0.5 | (2.95, 1.25) | (2.95, 0.75) |
