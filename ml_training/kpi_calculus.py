import pandas as pd
import numpy as np
import time
from sklearn.model_selection import train_test_split
from sklearn.ensemble import RandomForestClassifier
from sklearn.metrics import accuracy_score, f1_score
import warnings
warnings.filterwarnings('ignore')

# 1. Carregar les dades
print("Carregant Dades.csv...")
df = pd.read_csv('Dades.csv', sep=';')

# 2. Netejar i filtrar les classes amb poques mostres
counts = df['Class'].value_counts()
classes_bones = counts[counts > 20].index
df = df[df['Class'].isin(classes_bones)].copy()

# Separar columnes de text de les numèriques (espectres)
columnes_meta = ['Measure Type', 'Date', 'Time', 'Counter', 'Label', 'Class']
columnes_espectre = [col for col in df.columns if col not in columnes_meta]

# Convertir les comes a punts (si n'hi ha) per poder fer càlculs
for col in columnes_espectre:
    if df[col].dtype == 'object':
        df[col] = df[col].str.replace(',', '.').astype(float)

# Definir les X i les y
X = df[columnes_espectre]
y = df['Class']

# 3. Separar en Train i Test (80% / 20%)
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42, stratify=y)

# 4. Entrenar el model (Random Forest amb 100 arbres)
model = RandomForestClassifier(n_estimators=100, random_state=42)
model.fit(X_train, y_train)

# 5. Calcular l'Accuracy i l'F1-Score
y_pred = model.predict(X_test)

precisio = accuracy_score(y_test, y_pred)
f1 = f1_score(y_test, y_pred, average='weighted')

# 6. Calcular el temps d'inferència
# Fem la mitjana de 100 passades sobre 1 sola mostra per ser rigorosos
times = []
mostra_individual = X_test.iloc[0:1] # Agafem només 1 mostra

for i in range(100):
    start = time.time()
    _ = model.predict(mostra_individual)
    times.append(time.time() - start)

temps_inferencia_ms = np.mean(times) * 1000 # Convertir a mil·lisegons

# ==========================================
# IMPRIMIR ELS RESULTATS FINALS PELS KPIs
# ==========================================
print("\n" + "="*40)
print(" RESULTATS DELS KPIs (PER A L'ARTICLE)")
print("="*40)
print(f"1. Data Set Size:   {len(df)} mostres vàlides")
print(f"2. Global Accuracy: {precisio * 100:.2f}%")
print(f"3. F1-Score:        {f1:.4f}")
print(f"4. Inference Time:  {temps_inferencia_ms:.2f} ms")
print("="*40)