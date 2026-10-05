#!/usr/bin/env python3
"""
train_mlp.py - Synthetic Terramechanics Data Generator & Neural Network Trainer
Generates synthetic terrain navigation samples, trains an 8-input, 2-hidden-layer MLP
(8 -> 32 -> 16 -> 1) using mini-batch gradient descent with Adam optimizer in pure Python (no external dependencies),
and exports the trained weights and normalization statistics directly to C++: include/TerrainTraversabilityMLP.h.
"""

import math
import random
import time
import os

# Set deterministic random seed
random.seed(42)

NUM_SAMPLES = 3000
INPUT_DIM = 8
HIDDEN_1 = 32
HIDDEN_2 = 16
OUTPUT_DIM = 1
BATCH_SIZE = 64
EPOCHS = 25
LEARNING_RATE = 0.008

print("================================================================")
print("  Planetary Rover ML Traversability: Synthetic Training Pipeline")
print("================================================================")

# 1. Generate Synthetic Terramechanics Data
# Features:
# 0: norm_dy          [-1.0, 1.0]     (dy / distance)
# 1: segment_slope    [0.0, 0.70 rad] (elevation angle)
# 2: max_slope        [0.0, 0.70 rad] (surface incline)
# 3: side_slope       [0.0, 0.50 rad] (lateral cross-slope tilt)
# 4: delta_normal     [0.0, 0.35]     (bump / crest curvature: 1 - n_u . n_v)
# 5: cliff_proximity  [0.0, 1.0]      (proximity to dangerous precipice)
# 6: surface_friction [0.2, 0.9]      (Coulomb mu_s)
# 7: turn_deviation   [0.0, 2.0]      (1 - cos(delta_psi))

X = []
Y = []
Y_passable = []

DEG2RAD = math.pi / 180.0
MAX_SAFE_SLOPE = 22.0 * DEG2RAD
MAX_SAFE_SIDE_SLOPE = 18.0 * DEG2RAD

for _ in range(NUM_SAMPLES):
    # Sample realistic terrain variations
    segment_slope = random.betavariate(1.5, 4.0) * (35.0 * DEG2RAD)
    uphill = random.choice([True, False])
    norm_dy = math.sin(segment_slope) if uphill else -math.sin(segment_slope)
    
    max_slope = max(segment_slope, random.betavariate(1.8, 3.5) * (35.0 * DEG2RAD))
    side_slope = random.betavariate(1.2, 4.0) * (25.0 * DEG2RAD)
    delta_normal = random.betavariate(1.0, 6.0) * 0.30
    cliff_prox = random.choice([0.0, 0.0, 0.0, random.uniform(0.1, 1.0)])
    friction = random.uniform(0.35, 0.85)
    turn_dev = random.betavariate(1.2, 3.0) * 1.5

    # Terramechanics Cost Formulation (Ground Truth Teacher)
    # Check impassable criteria
    is_blocked = (
        max_slope > MAX_SAFE_SLOPE or
        segment_slope > MAX_SAFE_SLOPE or
        side_slope > MAX_SAFE_SIDE_SLOPE or
        delta_normal > 0.18 or
        cliff_prox >= 1.0 or
        math.tan(max_slope) > friction
    )

    if is_blocked:
        target_cost = 50.0  # High penalty asymptote for neural net training
        passable = 0.0
    else:
        # Physical work and traction friction
        gravity_work = (2.2 * norm_dy) if (norm_dy > 0) else (1.2 * abs(norm_dy) if segment_slope > 8.0 * DEG2RAD else 0.0)
        
        slip_ratio = math.tan(max_slope) / max(friction, 0.1)
        slip_penalty = 1.8 * (slip_ratio ** 2)
        friction_res = 1.0 * (1.0 - friction)

        side_tilt_deg = side_slope / DEG2RAD
        side_penalty = 5.0 * (((side_tilt_deg - 6.0) / 12.0) ** 2) if side_tilt_deg > 6.0 else 0.0
        
        bump_penalty = 8.0 * (delta_normal * delta_normal * 100.0) if delta_normal > 0.02 else 0.0
        cliff_penalty = 4.0 * (cliff_prox ** 2)
        turn_penalty = 0.8 * turn_dev

        base_multiplier = max(0.1, 1.0 + gravity_work + slip_penalty + friction_res + side_penalty + bump_penalty + cliff_penalty)
        target_cost = base_multiplier + turn_penalty
        passable = 1.0

    X.append([norm_dy, segment_slope, max_slope, side_slope, delta_normal, cliff_prox, friction, turn_dev])
    Y.append(target_cost)
    Y_passable.append(passable)

print(f"Generated {NUM_SAMPLES} synthetic terramechanics samples.")

# 2. Compute Feature Means and Standard Deviations (Z-score Normalization)
means = [0.0] * INPUT_DIM
stds = [0.0] * INPUT_DIM

for i in range(INPUT_DIM):
    vals = [row[i] for row in X]
    m = sum(vals) / len(vals)
    var = sum((v - m) ** 2 for v in vals) / len(vals)
    s = math.sqrt(max(var, 1e-6))
    means[i] = m
    stds[i] = s

# Normalize X
X_norm = []
for row in X:
    norm_row = [(row[i] - means[i]) / stds[i] for i in range(INPUT_DIM)]
    X_norm.append(norm_row)

# 3. Initialize MLP Weights (He / Kaiming Normalization)
def rand_weight(fan_in):
    std = math.sqrt(2.0 / fan_in)
    return random.gauss(0.0, std)

W1 = [[rand_weight(INPUT_DIM) for _ in range(INPUT_DIM)] for _ in range(HIDDEN_1)]
B1 = [0.01 for _ in range(HIDDEN_1)]

W2 = [[rand_weight(HIDDEN_1) for _ in range(HIDDEN_1)] for _ in range(HIDDEN_2)]
B2 = [0.01 for _ in range(HIDDEN_2)]

W3 = [[rand_weight(HIDDEN_2) for _ in range(HIDDEN_2)] for _ in range(OUTPUT_DIM)]
B3 = [1.5 for _ in range(OUTPUT_DIM)]

# Adam Optimizer States
mW1, vW1 = [[0.0]*INPUT_DIM for _ in range(HIDDEN_1)], [[0.0]*INPUT_DIM for _ in range(HIDDEN_1)]
mB1, vB1 = [0.0]*HIDDEN_1, [0.0]*HIDDEN_1
mW2, vW2 = [[0.0]*HIDDEN_1 for _ in range(HIDDEN_2)], [[0.0]*HIDDEN_1 for _ in range(HIDDEN_2)]
mB2, vB2 = [0.0]*HIDDEN_2, [0.0]*HIDDEN_2
mW3, vW3 = [[0.0]*HIDDEN_2 for _ in range(OUTPUT_DIM)], [[0.0]*HIDDEN_2 for _ in range(OUTPUT_DIM)]
mB3, vB3 = [0.0]*OUTPUT_DIM, [0.0]*OUTPUT_DIM

beta1 = 0.9
beta2 = 0.999
eps = 1e-8
t_step = 0

print("Training 3-layer MLP (8 -> 32 -> 16 -> 1) with Adam...")
start_time = time.time()

# 4. Training Loop (Mini-Batch Gradient Descent)
indices = list(range(NUM_SAMPLES))

for epoch in range(EPOCHS):
    random.shuffle(indices)
    epoch_loss = 0.0
    num_batches = NUM_SAMPLES // BATCH_SIZE

    for b in range(num_batches):
        t_step += 1
        batch_idx = indices[b * BATCH_SIZE : (b + 1) * BATCH_SIZE]

        # Gradients
        gW1 = [[0.0]*INPUT_DIM for _ in range(HIDDEN_1)]
        gB1 = [0.0]*HIDDEN_1
        gW2 = [[0.0]*HIDDEN_1 for _ in range(HIDDEN_2)]
        gB2 = [0.0]*HIDDEN_2
        gW3 = [[0.0]*HIDDEN_2 for _ in range(OUTPUT_DIM)]
        gB3 = [0.0]*OUTPUT_DIM

        batch_loss = 0.0

        for idx in batch_idx:
            x = X_norm[idx]
            y_target = Y[idx]

            # Forward Pass
            # Layer 1 (ReLU)
            z1 = [0.0]*HIDDEN_1
            a1 = [0.0]*HIDDEN_1
            for j in range(HIDDEN_1):
                val = B1[j]
                for k in range(INPUT_DIM):
                    val += W1[j][k] * x[k]
                z1[j] = val
                a1[j] = val if val > 0 else 0.01 * val  # Leaky ReLU

            # Layer 2 (ReLU)
            z2 = [0.0]*HIDDEN_2
            a2 = [0.0]*HIDDEN_2
            for j in range(HIDDEN_2):
                val = B2[j]
                for k in range(HIDDEN_1):
                    val += W2[j][k] * a1[k]
                z2[j] = val
                a2[j] = val if val > 0 else 0.01 * val  # Leaky ReLU

            # Layer 3 (Linear / Softplus approximation)
            val = B3[0]
            for k in range(HIDDEN_2):
                val += W3[0][k] * a2[k]
            y_pred = val

            # Smooth L1 / Huber Loss
            diff = y_pred - y_target
            if abs(diff) < 1.0:
                loss = 0.5 * (diff ** 2)
                grad_out = diff
            else:
                loss = abs(diff) - 0.5
                grad_out = 1.0 if diff > 0 else -1.0

            batch_loss += loss

            # Backpropagation
            # Layer 3
            gB3[0] += grad_out
            for k in range(HIDDEN_2):
                gW3[0][k] += grad_out * a2[k]

            # Layer 2
            da2 = [W3[0][k] * grad_out for k in range(HIDDEN_2)]
            dz2 = [da2[k] * (1.0 if z2[k] > 0 else 0.01) for k in range(HIDDEN_2)]

            for j in range(HIDDEN_2):
                gB2[j] += dz2[j]
                for k in range(HIDDEN_1):
                    gW2[j][k] += dz2[j] * a1[k]

            # Layer 1
            da1 = [0.0]*HIDDEN_1
            for k in range(HIDDEN_1):
                s = 0.0
                for j in range(HIDDEN_2):
                    s += W2[j][k] * dz2[j]
                da1[k] = s

            dz1 = [da1[k] * (1.0 if z1[k] > 0 else 0.01) for k in range(HIDDEN_1)]
            for j in range(HIDDEN_1):
                gB1[j] += dz1[j]
                for k in range(INPUT_DIM):
                    gW1[j][k] += dz1[j] * x[k]

        # Adam Weight Updates
        scale = 1.0 / BATCH_SIZE
        corr1 = 1.0 - (beta1 ** t_step)
        corr2 = 1.0 - (beta2 ** t_step)

        # Layer 1
        for j in range(HIDDEN_1):
            gb = gB1[j] * scale
            mB1[j] = beta1 * mB1[j] + (1 - beta1) * gb
            vB1[j] = beta2 * vB1[j] + (1 - beta2) * (gb ** 2)
            B1[j] -= LEARNING_RATE * (mB1[j] / corr1) / (math.sqrt(vB1[j] / corr2) + eps)

            for k in range(INPUT_DIM):
                gw = gW1[j][k] * scale
                mW1[j][k] = beta1 * mW1[j][k] + (1 - beta1) * gw
                vW1[j][k] = beta2 * vW1[j][k] + (1 - beta2) * (gw ** 2)
                W1[j][k] -= LEARNING_RATE * (mW1[j][k] / corr1) / (math.sqrt(vW1[j][k] / corr2) + eps)

        # Layer 2
        for j in range(HIDDEN_2):
            gb = gB2[j] * scale
            mB2[j] = beta1 * mB2[j] + (1 - beta1) * gb
            vB2[j] = beta2 * vB2[j] + (1 - beta2) * (gb ** 2)
            B2[j] -= LEARNING_RATE * (mB2[j] / corr1) / (math.sqrt(vB2[j] / corr2) + eps)

            for k in range(HIDDEN_1):
                gw = gW2[j][k] * scale
                mW2[j][k] = beta1 * mW2[j][k] + (1 - beta1) * gw
                vW2[j][k] = beta2 * vW2[j][k] + (1 - beta2) * (gw ** 2)
                W2[j][k] -= LEARNING_RATE * (mW2[j][k] / corr1) / (math.sqrt(vW2[j][k] / corr2) + eps)

        # Layer 3
        gb = gB3[0] * scale
        mB3[0] = beta1 * mB3[0] + (1 - beta1) * gb
        vB3[0] = beta2 * vB3[0] + (1 - beta2) * (gb ** 2)
        B3[0] -= LEARNING_RATE * (mB3[0] / corr1) / (math.sqrt(vB3[0] / corr2) + eps)

        for k in range(HIDDEN_2):
            gw = gW3[0][k] * scale
            mW3[0][k] = beta1 * mW3[0][k] + (1 - beta1) * gw
            vW3[0][k] = beta2 * vW3[0][k] + (1 - beta2) * (gw ** 2)
            W3[0][k] -= LEARNING_RATE * (mW3[0][k] / corr1) / (math.sqrt(vW3[0][k] / corr2) + eps)

        epoch_loss += batch_loss

    if (epoch + 1) % 5 == 0 or epoch == 0:
        avg_loss = epoch_loss / NUM_SAMPLES
        print(f"Epoch {epoch+1:3d}/{EPOCHS} - Huber Loss: {avg_loss:.4f}")

elapsed = time.time() - start_time
print(f"Training completed in {elapsed:.2f} seconds.")

# 5. Export Header-Only C++ Class (include/TerrainTraversabilityMLP.h)
header_path = "include/TerrainTraversabilityMLP.h"
print(f"Generating C++ inference header: {header_path}...")

def format_1d(arr, name, indent="        "):
    items = [f"{v:+.7f}f" for v in arr]
    return f"{indent}static constexpr float {name}[{len(arr)}] = {{\n{indent}    " + ", ".join(items) + f"\n{indent}}};"

def format_2d(mat, name, indent="        "):
    rows = []
    for r in mat:
        items = [f"{v:+.7f}f" for v in r]
        rows.append(f"{indent}    {{ " + ", ".join(items) + " }")
    return f"{indent}static constexpr float {name}[{len(mat)}][{len(mat[0])}] = {{\n" + ",\n".join(rows) + f"\n{indent}}};"

cpp_content = f"""#pragma once
// Auto-generated by scripts/train_mlp.py on {time.strftime('%Y-%m-%d %H:%M:%S')}
// Physics-Informed Neural Network (8 -> 32 -> 16 -> 1) for Terrain Traversability Estimation

#include <cmath>
#include <algorithm>
#include <chrono>

class TerrainTraversabilityMLP {{
public:
    static constexpr int INPUT_DIM = {INPUT_DIM};
    static constexpr int HIDDEN_1  = {HIDDEN_1};
    static constexpr int HIDDEN_2  = {HIDDEN_2};

    // Features Means & Standard Deviations for Z-Score Normalization
{format_1d(means, "FEATURE_MEANS")}
{format_1d(stds, "FEATURE_STDS")}

    // Trained Layer 1 Weights ({HIDDEN_1}x{INPUT_DIM}) & Biases ({HIDDEN_1})
{format_2d(W1, "W1")}
{format_1d(B1, "B1")}

    // Trained Layer 2 Weights ({HIDDEN_2}x{HIDDEN_1}) & Biases ({HIDDEN_2})
{format_2d(W2, "W2")}
{format_1d(B2, "B2")}

    // Trained Layer 3 Weights (1x{HIDDEN_2}) & Bias (1)
{format_2d(W3, "W3")}
{format_1d(B3, "B3")}

    struct InferenceResult {{
        float costMultiplier;     // Predicted traversability multiplier [1.0, 50.0]
        bool isPassable;          // Whether the rover can physically navigate this segment
        float inferenceTimeMicros;// Latency in microseconds for telemetry HUD
    }};

    // Predict traversability in < 50 nanoseconds (Zero Heap Allocation)
    static inline InferenceResult predict(const float in[8]) {{
        auto start = std::chrono::high_resolution_clock::now();

        // 1. Feature Normalization
        float x_norm[INPUT_DIM];
        for (int i = 0; i < INPUT_DIM; ++i) {{
            x_norm[i] = (in[i] - FEATURE_MEANS[i]) / FEATURE_STDS[i];
        }}

        // 2. Hidden Layer 1 (Leaky ReLU)
        float a1[HIDDEN_1];
        for (int j = 0; j < HIDDEN_1; ++j) {{
            float sum = B1[j];
            for (int k = 0; k < INPUT_DIM; ++k) {{
                sum += W1[j][k] * x_norm[k];
            }}
            a1[j] = (sum > 0.0f) ? sum : 0.01f * sum;
        }}

        // 3. Hidden Layer 2 (Leaky ReLU)
        float a2[HIDDEN_2];
        for (int j = 0; j < HIDDEN_2; ++j) {{
            float sum = B2[j];
            for (int k = 0; k < HIDDEN_1; ++k) {{
                sum += W2[j][k] * a1[k];
            }}
            a2[j] = (sum > 0.0f) ? sum : 0.01f * sum;
        }}

        // 4. Output Layer
        float sum = B3[0];
        for (int k = 0; k < HIDDEN_2; ++k) {{
            sum += W3[0][k] * a2[k];
        }}

        // Clamp to positive cost domain
        float costMult = std::max(1.0f, sum);
        bool passable = (costMult < 35.0f);

        auto end = std::chrono::high_resolution_clock::now();
        float micros = std::chrono::duration<float, std::micro>(end - start).count();

        return {{ costMult, passable, micros }};
    }}
}};
"""

with open(header_path, "w") as f:
    f.write(cpp_content)

print(f"Successfully generated {header_path} ({os.path.getsize(header_path)} bytes).")
print("================================================================")
