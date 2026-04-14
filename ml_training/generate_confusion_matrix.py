import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from sklearn.base import BaseEstimator, TransformerMixin
from sklearn.preprocessing import StandardScaler
from sklearn.ensemble import RandomForestClassifier
from sklearn.pipeline import Pipeline
from sklearn.model_selection import train_test_split
from sklearn.metrics import confusion_matrix

# ==========================================================
# 1. CLASSES DEL PIPELINE 11
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
# 2. CARREGA I PARTICIONS
# ==========================================================
print("Carregant dades...")
df = pd.read_csv('Dades.csv', sep=';', decimal=',')
counts = df['Class'].value_counts()
classes_bones = counts[counts > 20].index
df = df[df['Class'].isin(classes_bones)].copy()

meta_cols = ['Measure Type', 'Date', 'Time', 'Counter', 'Label', 'Class']
wv_cols = [c for c in df.columns if c not in meta_cols]
X = df[wv_cols].values.astype(float)
y = df['Class'].values

# Split exacte que vam fer a l'experiment original
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.20, stratify=y, random_state=42)

# ==========================================================
# 3. ENTRENAMENT DEL PIPELINE 11
# ==========================================================
print("Entrenant el Pipeline 11...")
pipe_11 = Pipeline([
    ('clamp', ClampingTransformer()), 
    ('trim', TrimmingTransformer()),
    ('snv', SNVTransformer()), 
    ('scaler', StandardScaler()), 
    ('rf', RandomForestClassifier(n_estimators=100, class_weight='balanced', random_state=42))
])

pipe_11.fit(X_train, y_train)
y_pred = pipe_11.predict(X_test)

# ==========================================================
# 4. GENERACIÓ DEL GRÀFIC
# ==========================================================
print("Generant imatge de la Matriu de Confusió...")
classes = np.unique(y) # Llista amb els noms reals
cm = confusion_matrix(y_test, y_pred, labels=classes)

plt.figure(figsize=(10, 8))
# Dibuixem el Heatmap amb colors blaus industrials
sns.heatmap(cm, annot=True, fmt='d', cmap='Blues', xticklabels=classes, yticklabels=classes)

plt.title('Matriu de Confusió - Pipeline 11 (Test Set)', fontsize=14, pad=20)
plt.xlabel('Predicció del Braç Robòtic', fontsize=12)
plt.ylabel('Material Real (Target)', fontsize=12)
plt.xticks(rotation=45, ha='right')

plt.tight_layout()
plt.savefig('grafic_3_matriu_confusio.png', dpi=300)
print(" -> Guardat 'grafic_3_matriu_confusio.png'")