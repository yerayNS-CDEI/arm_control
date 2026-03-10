import pandas as pd
import numpy as np
from sklearn.model_selection import train_test_split
from sklearn.ensemble import RandomForestClassifier
from sklearn.metrics import classification_report, accuracy_score
import pickle
import warnings
warnings.filterwarnings('ignore') # Amagar avisos menors de Pandas

print("1. Carregant la base de dades 'Dades.csv'...")
df = pd.read_csv('Dades.csv', sep=';')

print("2. Netejant i filtrant dades...")
# a) Quedar-nos només amb les classes que tenen més de 20 mostres (eliminar classes de 1 sola mostra)
counts = df['Class'].value_counts()
classes_bones = counts[counts > 20].index
df = df[df['Class'].isin(classes_bones)].copy()

# b) Separar meta-dades (el text) dels valors purs de la càmera (els espectres)
columnes_meta = ['Measure Type', 'Date', 'Time', 'Counter', 'Label', 'Class']
columnes_espectre = [col for col in df.columns if col not in columnes_meta]

# c) Canviar les comes ',' per punts '.' en tots els valors numèrics i passar a Float
for col in columnes_espectre:
    if df[col].dtype == 'object':
        df[col] = df[col].str.replace(',', '.').astype(float)

# Creem X (dades) i y (etiqueta a predir)
y = df['Class']
X = df[columnes_espectre]

print(f"-> Ens hem quedat amb {len(df)} mostres vàlides de {len(y.unique())} materials diferents.")

print("\n3. Dividint les dades en Entrenament (80%) i Test (20%)...")
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42, stratify=y)

print("4. Entrenant el model d'Intel·ligència Artificial (Random Forest)...")
model = RandomForestClassifier(n_estimators=100, random_state=42)
model.fit(X_train, y_train)

print("\n5. Avaluant el model (Resultats del Test)...")
y_pred = model.predict(X_test)
precisio = accuracy_score(y_test, y_pred)
print("="*50)
print(f" EXACTITUD GLOBAL DEL MODEL: {precisio * 100:.2f}%")
print("="*50)
print("\nInforme detallat per cada material:")
print(classification_report(y_test, y_pred))

print("\n6. Guardant el model definitiu...")
with open('model_entrenat.pkl', 'wb') as f:
    pickle.dump(model, f)

print(" EXCEL·LENT! El fitxer 'model_entrenat.pkl' s'ha creat correctament.")