import pandas as pd
import numpy as np
from scipy.interpolate import interp1d
import os

# 1. Detectar automàticament a quina carpeta està guardat AQUEST script
directori_actual = os.path.dirname(os.path.abspath(__file__))

# 2. Construir les rutes exactes on anar a buscar i guardar els arxius
ruta_entrada = os.path.join(directori_actual, 'Dades.csv')
ruta_sortida_5 = os.path.join(directori_actual, 'Dades_interp_5.csv')
ruta_sortida_10 = os.path.join(directori_actual, 'Dades_interp_10.csv')

# Carregar les dades (ara sí, sap exactament on buscar)
df = pd.read_csv(ruta_entrada, sep=';', decimal=',')

# Identificar les columnes de metadades i les de longitud d'ona
meta_cols = ['Measure Type', 'Date', 'Time', 'Counter', 'Label', 'Class']
wv_cols = [c for c in df.columns if c not in meta_cols]

# Convertir els noms de les columnes de longitud d'ona a valors numèrics
wv_floats = np.array([float(str(c).replace(',', '.')) for c in wv_cols])

# Crear els dos rangs de longituds d'ona desitjats
wvs_5_range1 = np.arange(325, 795 + 5, 5)
wvs_5_range2 = np.arange(990, 1710 + 5, 5)
wvs_5 = np.concatenate([wvs_5_range1, wvs_5_range2])

wvs_10_range1 = np.arange(325, 795 + 10, 10)
wvs_10_range2 = np.arange(990, 1710 + 10, 10)
wvs_10 = np.concatenate([wvs_10_range1, wvs_10_range2])

data_5 = []
data_10 = []

for i, row in df.iterrows():
    meta = row[meta_cols].to_dict()
    y = row[wv_cols].values.astype(float)
    
    f = interp1d(wv_floats, y, kind='linear', fill_value='extrapolate')
    
    y_5 = f(wvs_5)
    y_10 = f(wvs_10)
    
    y_5 = np.maximum(0, y_5)
    y_10 = np.maximum(0, y_10)

    row_5 = meta.copy()
    row_5.update({str(int(w)): y_5[j] for j, w in enumerate(wvs_5)})
    data_5.append(row_5)
    
    row_10 = meta.copy()
    row_10.update({str(int(w)): y_10[j] for j, w in enumerate(wvs_10)})
    data_10.append(row_10)

df_5 = pd.DataFrame(data_5)
df_10 = pd.DataFrame(data_10)

# Guardar els nous DataFrames a la MATEIXA carpeta on és l'script
df_5.to_csv(ruta_sortida_5, sep=';', decimal=',', index=False)
df_10.to_csv(ruta_sortida_10, sep=';', decimal=',', index=False)

print("Els arxius s'han generat correctament a la carpeta:", directori_actual)