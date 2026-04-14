import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os
from sklearn.base import BaseEstimator, TransformerMixin
from sklearn.preprocessing import StandardScaler
from sklearn.ensemble import RandomForestClassifier

# ==========================================================
# 1. CLASSES NECESSÀRIES (Pipeline 11)
# ==========================================================
class TrimmingTransformer(BaseEstimator, TransformerMixin):
    def __init__(self, cut_front=10, cut_back=10):
        self.cut_front = cut_front
        self.cut_back = cut_back
    def fit(self, X, y=None): return self
    def transform(self, X):
        vis_good = X[:, self.cut_front : 256 - self.cut_back]
        nir_good = X[:, 256 + self.cut_front : 512 - self.cut_back]
        return np.hstack((vis_good, nir_good))

class ClampingTransformer(BaseEstimator, TransformerMixin):
    def fit(self, X, y=None): return self
    def transform(self, X): return np.clip(X, 0.0, None)

class SNVTransformer(BaseEstimator, TransformerMixin):
    def fit(self, X, y=None): return self
    def transform(self, X):
        mean = np.mean(X, axis=1, keepdims=True)
        std = np.std(X, axis=1, keepdims=True)
        return np.divide((X - mean), std, out=np.zeros_like(X), where=std > 0)

# ==========================================================
# 2. CARREGA DE DADES
# ==========================================================
print("Carregant dades...")
df = pd.read_csv('Dades.csv', sep=';', decimal=',')
counts = df['Class'].value_counts()
classes_bones = counts[counts > 20].index
df = df[df['Class'].isin(classes_bones)].copy()

meta_cols = ['Measure Type', 'Date', 'Time', 'Counter', 'Label', 'Class']
wv_cols = [c for c in df.columns if c not in meta_cols]
X_raw = df[wv_cols].values.astype(float)
y = df['Class'].values

# Longituds d'ona originals (512 punts)
wv_floats = np.linspace(325, 1707, 512)

# ==========================================================
# 3. GRÀFIC 1: EFECTE FÍSIC (ABANS I DESPRÉS)
# ==========================================================
print("Generant Gràfic 1: Efecte de l'SNV i Trimming...")
# Triem una classe aleatòria per a la visualització (la primera que tinguem)
target_class = classes_bones[0] 
idx = np.where(y == target_class)[0][:10] # Agafem 10 mostres aleatòries
X_sample = X_raw[idx]

# Apliquem el pre-processament pas a pas només per a visualitzar
X_clamp = ClampingTransformer().transform(X_sample)
X_trim = TrimmingTransformer().transform(X_clamp)
X_snv = SNVTransformer().transform(X_trim)

# Calculem l'eix X (les longituds d'ona que queden després del trimming)
wv_vis_good = wv_floats[10 : 256 - 10]
wv_nir_good = wv_floats[256 + 10 : 512 - 10]
wv_final = np.concatenate([wv_vis_good, wv_nir_good])

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))

# Plot Abans
for i in range(10):
    ax1.plot(wv_floats, X_sample[i], color='red', alpha=0.6)
ax1.set_title(f'Dades Crues ({target_class})\n(Soroll tèrmic i Scattering visibles)')
ax1.set_xlabel('Longitud d\'ona (nm)')
ax1.set_ylabel('Reflectància Bruta')
ax1.grid(True, alpha=0.3)

# Plot Després
for i in range(10):
    ax2.plot(wv_final, X_snv[i], color='green', alpha=0.7)
ax2.set_title(f'Després del Pipeline 11 (SNV + Trimming)\n(Signatura química aïllada)')
ax2.set_xlabel('Longitud d\'ona (nm)')
ax2.set_ylabel('Reflectància Normalitzada')
ax2.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('grafic_1_abans_despres.png', dpi=300)
print(" -> Guardat 'grafic_1_abans_despres.png'")

# ==========================================================
# 4. GRÀFIC 2: FEATURE IMPORTANCE (ON MIRA EL ROBOT?)
# ==========================================================
print("Entrenant Random Forest per analitzar la importància...")
# Passem TOTA la base de dades pel pipeline complet
X_c = ClampingTransformer().transform(X_raw)
X_t = TrimmingTransformer().transform(X_c)
X_s = SNVTransformer().transform(X_t)
X_scaled = StandardScaler().fit_transform(X_s)

# Entrenem el model definitiu
rf = RandomForestClassifier(n_estimators=100, class_weight='balanced', random_state=42)
rf.fit(X_scaled, y)

importances = rf.feature_importances_

plt.figure(figsize=(12, 5))
# Omplim de blau la part Visible (VIS) i de vermell l'Infraroig (NIR)
plt.bar(wv_final[wv_final < 1000], importances[wv_final < 1000], color='blue', alpha=0.7, label='Sensor VIS (<1000nm)', width=4)
plt.bar(wv_final[wv_final >= 1000], importances[wv_final >= 1000], color='darkred', alpha=0.7, label='Sensor NIR (>1000nm)', width=4)

plt.title('Importància de Característiques (Feature Importance)\nQuines longituds d\'ona fan servir els arbres per classificar?')
plt.xlabel('Longitud d\'ona (nm)')
plt.ylabel('Importància Relativa (Gini)')
plt.legend()
plt.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('grafic_2_feature_importance.png', dpi=300)
print(" -> Guardat 'grafic_2_feature_importance.png'")
print("\n¡PROCÉS FINALITZAT! Revisa les dues imatges creades a la teva carpeta.")