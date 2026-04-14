import pandas as pd
import numpy as np
import pickle
from sklearn.base import BaseEstimator, TransformerMixin
from sklearn.preprocessing import StandardScaler
from sklearn.ensemble import RandomForestClassifier
from sklearn.pipeline import Pipeline

# =====================================================================
# 1. CLASSES DEL PRE-PROCESSAMENT (Lògica Física)
# =====================================================================
class TrimmingTransformer(BaseEstimator, TransformerMixin):
    def __init__(self, cut_front=10, cut_back=10):
        self.cut_front = cut_front
        self.cut_back = cut_back
        
    def fit(self, X, y=None): 
        return self
        
    def transform(self, X):
        vis_good = X[:, self.cut_front : 256 - self.cut_back]
        nir_good = X[:, 256 + self.cut_front : 512 - self.cut_back]
        return np.hstack((vis_good, nir_good))

class ClampingTransformer(BaseEstimator, TransformerMixin):
    def fit(self, X, y=None): 
        return self
        
    def transform(self, X): 
        return np.clip(X, 0.0, None)

class SNVTransformer(BaseEstimator, TransformerMixin):
    def fit(self, X, y=None): 
        return self
        
    def transform(self, X):
        mean = np.mean(X, axis=1, keepdims=True)
        std = np.std(X, axis=1, keepdims=True)
        # S'evita la divisió per zero de forma segura
        return np.divide((X - mean), std, out=np.zeros_like(X), where=std > 0)

# =====================================================================
# 2. CARREGA DE DADES (100% del Dataset)
# =====================================================================
print("1. Llegint la base de dades 'Dades.csv'...")
df = pd.read_csv('Dades.csv', sep=';', decimal=',')

print("2. Aplicant filtres de qualitat...")
# Eliminem materials minoritaris (menys de 20 captures)
counts = df['Label'].value_counts()
classes_bones = counts[counts > 20].index
df = df[df['Class'].isin(classes_bones)].copy()

# Separem variables
meta_cols = ['Measure Type', 'Date', 'Time', 'Counter', 'Label', 'Class']
wv_cols = [c for c in df.columns if c not in meta_cols]

X = df[wv_cols].values.astype(float)
y = df['Class'].values

print(f" -> Utilitzarem {len(X)} mostres per a l'entrenament final.")
print(f" -> Classes detectades: {list(classes_bones)}")

# =====================================================================
# 3. CONSTRUCCIÓ I ENTRENAMENT DE L'ARQUITECTURA
# =====================================================================
print("\n3. Construint el Pipeline Definitiu (Arquitectura 11)...")
# Aquí definim la cadena exacta que connectarà amb ROS 2
pipeline_produccio = Pipeline([
    ('clamp', ClampingTransformer()),             # 1. Talla reflexos negatius
    ('trim', TrimmingTransformer()),              # 2. Elimina soroll dels extrems
    ('snv', SNVTransformer()),                    # 3. Corregeix l'efecte Scattering i distància
    ('scaler', StandardScaler()),                 # 4. Estandarditza les magnituds
    ('rf', RandomForestClassifier(                # 5. El cervell
        n_estimators=100, 
        class_weight='balanced', 
        random_state=42
    ))
])

print("4. Entrenant el model amb el 100% de les dades (Això pot trigar uns segons)...")
pipeline_produccio.fit(X, y)

# =====================================================================
# 4. EXPORTACIÓ DEL MODEL PER A ROS 2
# =====================================================================
nom_fitxer = 'model_entrenat_pipeline11.pkl'

print(f"\n5. Guardant el model compilat a '{nom_fitxer}'...")
with open(nom_fitxer, 'wb') as f:
    pickle.dump(pipeline_produccio, f)

print("="*60)
print(" EXCEL·LENT! El model de producció s'ha creat correctament.")
print(f" Ara ja pots integrar '{nom_fitxer}' al teu node de ROS 2.")
print("="*60)