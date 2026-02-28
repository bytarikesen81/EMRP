# -*- coding: utf-8 -*-
"""
Training script for a tiny 1D-CNN on (96 x 6) IMU windows.
Dataset format: pre-windowed CSVs with columns label, window_index, x0_ax ... x95_gz

- Classes      : Move, Rest, Shake  (3-class)
- Window size  : 96 samples @ 100 Hz  (~0.96 s)
- Features     : aX, aY, aZ, gX, gY, gZ  (6 channels, raw int16)
- Preprocess   : per-channel z-score (fit on TRAIN only) + clip [-5, 5]
- Model        : Conv1D(16,k=5) -> Conv1D(16,k=3) -> MaxPool -> Conv1D(24,k=3)
                 -> GAP -> Dense(24) -> Dropout(0.30) -> Softmax(3)
                 + GaussianNoise(0.02) at input (training-time only)
- EarlyStopping: monitor val_loss, patience=25, restore_best_weights
- Export       : gesture_model.tflite (float32) + normalization.json
"""

import os
import json
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import tensorflow as tf
from tensorflow.keras import layers, models

try:
    from sklearn.metrics import confusion_matrix, ConfusionMatrixDisplay
    SKLEARN_OK = True
except ImportError:
    SKLEARN_OK = False

print(f"TensorFlow version = {tf.__version__}\n")

# ─────────────────────────────────────────────────────────────────────────────
# 0) Reproducibility & constants
# ─────────────────────────────────────────────────────────────────────────────
SEED = 1337
np.random.seed(SEED)
tf.random.set_seed(SEED)

GESTURES           = ["Move", "Rest", "Shake"]   # must match CSV label values exactly
NUM_GESTURES       = len(GESTURES)
SAMPLES_PER_GESTURE = 96
FEATS              = 6                            # aX, aY, aZ, gX, gY, gZ

# ── Paths ────────────────────────────────────────────────────────────────────
# Put your three clean CSV files in DATA_DIR, or adjust paths below.
DATA_DIR = "../server/model/dataset"         # folder containing Move_clean.csv etc.
OUT_DIR  = "model_output"
os.makedirs(OUT_DIR, exist_ok=True)

DATASET_FILES = {
    "Move":  os.path.join(DATA_DIR, "Move_clean.csv"),
    "Rest":  os.path.join(DATA_DIR, "Rest_clean.csv"),
    "Shake": os.path.join(DATA_DIR, "Shake_clean.csv"),
}

# ─────────────────────────────────────────────────────────────────────────────
# 1) Load pre-windowed CSVs
#    Each row = one window.  Columns: label, window_index, x0_ax ... x95_gz
#    We reshape each row into (96, 6) = [aX, aY, aZ, gX, gY, gZ] per time step.
# ─────────────────────────────────────────────────────────────────────────────
CHANNEL_SUFFIXES = ["ax", "ay", "az", "gx", "gy", "gz"]

ONE_HOT  = np.eye(NUM_GESTURES, dtype=np.float32)
inputs_list, outputs_list = [], []

for g_idx, gesture in enumerate(GESTURES):
    path = DATASET_FILES[gesture]
    df   = pd.read_csv(path)

    # Build ordered feature columns: x0_ax, x0_ay, ..., x95_gz
    feat_cols = []
    for t in range(SAMPLES_PER_GESTURE):
        for ch in CHANNEL_SUFFIXES:
            feat_cols.append(f"x{t}_{ch}")

    windows = df[feat_cols].values.astype(np.float32)   # (N, 576)
    labels  = np.tile(ONE_HOT[g_idx], (len(windows), 1))

    inputs_list.append(windows)
    outputs_list.append(labels)
    print(f"  Loaded '{gesture}': {len(windows)} windows  (file: {path})")

inputs  = np.concatenate(inputs_list,  axis=0)   # (N_total, 576)
outputs = np.concatenate(outputs_list, axis=0)   # (N_total, 3)
print(f"\nTotal windows: {len(inputs)} | Input shape: {inputs.shape}")

# ─────────────────────────────────────────────────────────────────────────────
# 2) Shuffle + split  60 / 20 / 20
# ─────────────────────────────────────────────────────────────────────────────
idx = np.random.permutation(len(inputs))
inputs  = inputs[idx]
outputs = outputs[idx]

n       = len(inputs)
n_train = int(0.60 * n)
n_val   = int(0.20 * n)

X_flat_train = inputs[:n_train]
X_flat_val   = inputs[n_train : n_train + n_val]
X_flat_test  = inputs[n_train + n_val:]

y_train = outputs[:n_train]
y_val   = outputs[n_train : n_train + n_val]
y_test  = outputs[n_train + n_val:]

print(f"Train / Val / Test = {len(X_flat_train)} / {len(X_flat_val)} / {len(X_flat_test)}")

# ─────────────────────────────────────────────────────────────────────────────
# 3) Per-channel z-score normalisation (fit on TRAIN only)
#    Shape trick: flatten -> (N, 96, 6) -> compute mean/std per channel axis
# ─────────────────────────────────────────────────────────────────────────────
train_ts = X_flat_train.reshape(-1, SAMPLES_PER_GESTURE, FEATS)  # (N, 96, 6)
ch_mean  = train_ts.mean(axis=(0, 1), keepdims=True)             # (1, 1, 6)
ch_std   = train_ts.std( axis=(0, 1), keepdims=True) + 1e-8      # (1, 1, 6)

def zscore(x_flat: np.ndarray) -> np.ndarray:
    """Reshape to (N, 96, 6), apply per-channel z-score, clip, return (N, 96, 6)."""
    x_ts = x_flat.reshape(-1, SAMPLES_PER_GESTURE, FEATS)
    x_ts = (x_ts - ch_mean) / ch_std
    x_ts = np.clip(x_ts, -5.0, 5.0)
    return x_ts

X_train = zscore(X_flat_train)   # (N, 96, 6)
X_val   = zscore(X_flat_val)
X_test  = zscore(X_flat_test)

print(f"X_train: {X_train.shape} | X_val: {X_val.shape} | X_test: {X_test.shape}")
rng = np.random.default_rng(SEED)

noise  = rng.normal(0, 0.05, X_train.shape).astype(np.float32)
scale  = rng.uniform(0.9, 1.1, (len(X_train), 1, 1)).astype(np.float32)
X_aug  = np.clip(X_train * scale + noise, -5.0, 5.0)
y_aug  = y_train.copy()

X_train = np.concatenate([X_train, X_aug], axis=0)
y_train = np.concatenate([y_train, y_aug], axis=0)


shuffle_idx = rng.permutation(len(X_train))
X_train = X_train[shuffle_idx]
y_train = y_train[shuffle_idx]

print(f"Augmentation sonrası Train: {len(X_train)} pencere (2x)")
# ─────────────────────────────────────────────────────────────────────────────
# 4) Model definition  — tiny 1D-CNN
# ─────────────────────────────────────────────────────────────────────────────
def build_model(input_shape=(SAMPLES_PER_GESTURE, FEATS), num_classes=NUM_GESTURES):
    inp = layers.Input(shape=input_shape, name="imu_96x6")

    # GaussianNoise: active only during training, disabled at inference
    x = layers.GaussianNoise(0.02)(inp)

    x = layers.Conv1D(16, 5, padding="same", activation="relu")(x)
    x = layers.BatchNormalization()(x)   # <-- EKLENDİ

    x = layers.Conv1D(16, 3, padding="same", activation="relu")(x)
    x = layers.BatchNormalization()(x)   # <-- EKLENDİ

    x = layers.MaxPooling1D(2)(x)

    x = layers.Conv1D(24, 3, padding="same", activation="relu")(x)
    x = layers.BatchNormalization()(x)   # <-- EKLENDİ
    x = layers.GlobalAveragePooling1D()(x)
    x = layers.Dense(24, activation="relu")(x)
    x = layers.Dropout(0.30)(x)                  # increased from 0.10 -> 0.30 (small dataset)
    out = layers.Dense(num_classes, activation="softmax")(x)

    model = models.Model(inp, out, name="cnn_imu_3class")
    model.compile(
        optimizer=tf.keras.optimizers.Adam(1e-3),
        loss="categorical_crossentropy",
        metrics=["accuracy"]
    )
    return model

model = build_model()
model.summary()

# ─────────────────────────────────────────────────────────────────────────────
# 5) Training
# ─────────────────────────────────────────────────────────────────────────────
early = tf.keras.callbacks.EarlyStopping(
    monitor="val_loss",
    patience=25,
    restore_best_weights=True
)

# YENİ: ReduceLROnPlateau
reduce_lr = tf.keras.callbacks.ReduceLROnPlateau(
    monitor="val_loss",
    factor=0.5,
    patience=10,
    min_lr=1e-6,
    verbose=1
)

history = model.fit(
    X_train, y_train,
    validation_data=(X_val, y_val),
    epochs=200,
    batch_size=8,
    callbacks=[early, reduce_lr],   # <-- reduce_lr eklendi
    verbose=2
)

# ─────────────────────────────────────────────────────────────────────────────
# 6) Learning curves
# ─────────────────────────────────────────────────────────────────────────────
fig, axes = plt.subplots(1, 2, figsize=(14, 5))

axes[0].plot(history.history["loss"],     "g.-", label="Train loss")
axes[0].plot(history.history["val_loss"], "b.-", label="Val loss")
axes[0].set_title("Loss"); axes[0].set_xlabel("Epoch")
axes[0].grid(True); axes[0].legend()

axes[1].plot(history.history["accuracy"],     "g.-", label="Train acc")
axes[1].plot(history.history["val_accuracy"], "b.-", label="Val acc")
axes[1].set_title("Accuracy"); axes[1].set_xlabel("Epoch")
axes[1].grid(True); axes[1].legend()

plt.tight_layout()
plt.savefig(os.path.join(OUT_DIR, "training_curves.png"), dpi=150)
plt.show()

# ─────────────────────────────────────────────────────────────────────────────
# 7) Test evaluation + confusion matrix
# ─────────────────────────────────────────────────────────────────────────────
test_loss, test_acc = model.evaluate(X_test, y_test, verbose=0)
best_val_acc  = float(np.max(history.history.get("val_accuracy", [np.nan])))
best_val_loss = float(np.min(history.history.get("val_loss",     [np.nan])))

print(f"\nBest val_acc  = {best_val_acc:.4f} | Best val_loss = {best_val_loss:.4f}")
print(f"Test accuracy = {test_acc:.4f}     | Test loss     = {test_loss:.4f}")

y_prob = model.predict(X_test, verbose=0)
y_pred = y_prob.argmax(axis=1)
y_true = y_test.argmax(axis=1)

if SKLEARN_OK:
    cm   = confusion_matrix(y_true, y_pred, labels=list(range(NUM_GESTURES)))
    disp = ConfusionMatrixDisplay(confusion_matrix=cm, display_labels=GESTURES)
    disp.plot(cmap="Blues", values_format="d")
    plt.title("Confusion Matrix (Test Set)")
    plt.savefig(os.path.join(OUT_DIR, "confusion_matrix.png"), dpi=150)
    plt.show()
    print("\nConfusion matrix:")
    print(cm)
else:
    print("scikit-learn not found — skipping confusion matrix.")

# Class probabilities over test windows (sanity check)
plt.figure(figsize=(14, 4))
for c_idx, cname in enumerate(GESTURES):
    plt.plot(y_prob[:, c_idx], ".-", label=f"P({cname})")
plt.title("Class probabilities — test windows")
plt.xlabel("Window index"); plt.ylabel("Probability")
plt.grid(True); plt.legend(); plt.show()

# ─────────────────────────────────────────────────────────────────────────────
# 8) Export: TFLite (float32) + normalization.json
# ─────────────────────────────────────────────────────────────────────────────
# --- TFLite ---
converter    = tf.lite.TFLiteConverter.from_keras_model(model)
tflite_bytes = converter.convert()

tflite_path = os.path.join(OUT_DIR, "gesture_model.tflite")
with open(tflite_path, "wb") as f:
    f.write(tflite_bytes)
print(f"\nTFLite saved: {tflite_path}  ({os.path.getsize(tflite_path):,} bytes)")

# --- normalization.json ---
# The model server MUST apply the same per-channel z-score before inference.
# Load this file on the RPi and apply:
#   x_norm = (x_raw - channel_mean) / channel_std  then clip to [-5, 5]
norm = {
    "mode":         "channelwise_zscore",
    "channels":     CHANNEL_SUFFIXES,          # ["ax","ay","az","gx","gy","gz"]
    "channel_mean": ch_mean.reshape(-1).tolist(),   # 6 values
    "channel_std":  ch_std.reshape(-1).tolist(),    # 6 values
    "clip":         [-5.0, 5.0],
    "window_size":  SAMPLES_PER_GESTURE,
    "sample_rate_hz": 100,
    "label_map":    {str(i): g for i, g in enumerate(GESTURES)}
}

norm_path = os.path.join(OUT_DIR, "normalization.json")
with open(norm_path, "w") as f:
    json.dump(norm, f, indent=2)
print(f"Normalization saved: {norm_path}")

# ─────────────────────────────────────────────────────────────────────────────
# 9) Quick inference sanity check
#    Verifies the exported TFLite model gives identical outputs to Keras.
# ─────────────────────────────────────────────────────────────────────────────
interp = tf.lite.Interpreter(model_path=tflite_path)
interp.allocate_tensors()
inp_det  = interp.get_input_details()[0]
out_det  = interp.get_output_details()[0]

sample   = X_test[0:1].astype(np.float32)
keras_out = model.predict(sample, verbose=0)

interp.set_tensor(inp_det["index"], sample)
interp.invoke()
tflite_out = interp.get_tensor(out_det["index"])

print(f"\nSanity check (window 0):")
print(f"  Keras  : {keras_out[0]}")
print(f"  TFLite : {tflite_out[0]}")
print(f"  Max diff: {np.abs(keras_out - tflite_out).max():.2e}")
print("\nTraining complete.")
