import numpy as np
from sklearn.base import BaseEstimator, TransformerMixin
from scipy.signal import savgol_filter
from scipy.ndimage import median_filter
from scipy.interpolate import interp1d
import pywt  # Necessari per al Wavelet Denoising (pip install PyWavelets)
import time
from sklearn.metrics import (confusion_matrix, accuracy_score, precision_score, 
                             recall_score, f1_score, log_loss, cohen_kappa_score, roc_auc_score)
from sklearn.pipeline import Pipeline
from sklearn.preprocessing import StandardScaler
from sklearn.decomposition import PCA
from sklearn.linear_model import LogisticRegression
from sklearn.feature_selection import SelectKBest, f_classif
from sklearn.cross_decomposition import PLSRegression
from sklearn.base import BaseEstimator, TransformerMixin
import pandas as pd
import os
from sklearn.model_selection import train_test_split
from sklearn.ensemble import RandomForestClassifier
from sklearn.model_selection import cross_val_score, StratifiedKFold

# =====================================================================
# MOTOR DE PRE-PROCESSAMENT HIPERESPECTRAL (CUSTOM TRANSFORMERS)
# =====================================================================

class TrimmingTransformer(BaseEstimator, TransformerMixin):
    """
    Retalla els extrems morts (soroll pur) dels sensors VIS i NIR.
    Assumim un array d'entrada de 512 variables (0:256 VIS, 256:512 NIR).
    """
    def __init__(self, cut_front=10, cut_back=10):
        self.cut_front = cut_front
        self.cut_back = cut_back

    def fit(self, X, y=None):
        return self

    def transform(self, X):
        # Tallem de manera independent per a cada sensor
        vis_good = X[:, self.cut_front : 256 - self.cut_back]
        nir_good = X[:, 256 + self.cut_front : 512 - self.cut_back]
        return np.hstack((vis_good, nir_good))


class ClampingTransformer(BaseEstimator, TransformerMixin):
    """
    Físicament, la reflectància no pot ser negativa. 
    Això retalla a 0 qualsevol valor negatiu fruit del soroll tèrmic.
    """
    def fit(self, X, y=None):
        return self

    def transform(self, X):
        return np.clip(X, 0.0, None)


class MedianFilterTransformer(BaseEstimator, TransformerMixin):
    """
    Elimina pics anòmals ("Salt and Pepper" noise) del sensor VIS 
    provocats per inestabilitats TCP sense difuminar la química.
    """
    def __init__(self, window=3):
        self.window = window

    def fit(self, X, y=None):
        return self

    def transform(self, X):
        # S'aplica només a l'eix 1 (al llarg de l'espectre, no entre mostres)
        return median_filter(X, size=(1, self.window), mode='nearest')


class SavitzkyGolayTransformer(BaseEstimator, TransformerMixin):
    """
    Suavitza el soroll de l'halògena respectant la forma dels pics.
    Pot calcular la 1a o 2a derivada si se li demana.
    """
    def __init__(self, window_length=11, polyorder=2, deriv=0):
        self.window_length = window_length
        self.polyorder = polyorder
        self.deriv = deriv

    def fit(self, X, y=None):
        return self

    def transform(self, X):
        return savgol_filter(X, window_length=self.window_length, 
                             polyorder=self.polyorder, deriv=self.deriv, axis=1)


class SNVTransformer(BaseEstimator, TransformerMixin):
    """
    Standard Normal Variate. Corregeix l'efecte de distància geomètrica (Scattering).
    Sense estat (Stateless) - ideal per a ROS 2 en temps real.
    """
    def fit(self, X, y=None):
        return self

    def transform(self, X):
        mean = np.mean(X, axis=1, keepdims=True)
        std = np.std(X, axis=1, keepdims=True)
        # Regla estricta d'arquitectura: Evitar divisions per zero de manera segura
        return np.divide((X - mean), std, out=np.zeros_like(X), where=std > 0)


class MSCTransformer(BaseEstimator, TransformerMixin):
    """
    Multiplicative Scatter Correction. 
    Amb estat (Stateful): Aprèn l'espectre ideal durant l'entrenament.
    """
    def __init__(self):
        self.mean_spectrum_ = None

    def fit(self, X, y=None):
        # Calculem l'espectre mitjà ideal de tot el dataset d'entrenament
        self.mean_spectrum_ = np.mean(X, axis=0)
        return self

    def transform(self, X):
        if self.mean_spectrum_ is None:
            raise RuntimeError("MSC no ha estat entrenat (crida a .fit() primer).")
            
        X_msc = np.zeros_like(X)
        for i in range(X.shape[0]):
            # Regressió lineal de la mostra actual contra l'espectre ideal
            fit = np.polyfit(self.mean_spectrum_, X[i, :], 1)
            # fit[0] és el pendent (multiplicatiu), fit[1] és l'offset (additiu)
            if fit[0] != 0:
                X_msc[i, :] = (X[i, :] - fit[1]) / fit[0]
            else:
                X_msc[i, :] = X[i, :] # Seguretat extrema
        return X_msc


class DetrendingTransformer(BaseEstimator, TransformerMixin):
    """
    Elimina la 'panxa' tèrmica provocada per la bombeta halògena 
    ajustant una paràbola de grau 2 i restant-la.
    """
    def fit(self, X, y=None):
        return self

    def transform(self, X):
        X_detrended = np.zeros_like(X)
        x_axis = np.arange(X.shape[1])
        for i in range(X.shape[0]):
            p = np.polyfit(x_axis, X[i], 2)
            baseline = np.polyval(p, x_axis)
            X_detrended[i] = X[i] - baseline
        return X_detrended


class InterpolationTransformer(BaseEstimator, TransformerMixin):
    def __init__(self, original_wvs, target_wvs):
        # Ara NO ho toquem aquí per complir les regles de Scikit-Learn
        self.original_wvs = original_wvs
        self.target_wvs = target_wvs

    def fit(self, X, y=None):
        return self

    def transform(self, X):
        # Convertim a numpy array JUST AL MOMENT de fer la transformació
        orig_arr = np.array(self.original_wvs)
        targ_arr = np.array(self.target_wvs)
        
        X_interp = np.zeros((X.shape[0], len(targ_arr)))
        for i in range(X.shape[0]):
            f = interp1d(orig_arr, X[i, :], kind='linear', fill_value='extrapolate')
            X_interp[i, :] = np.maximum(0, f(targ_arr))
        return X_interp


class WaveletDenoisingTransformer(BaseEstimator, TransformerMixin):
    """
    Neteja el soroll d'alta freqüència mitjançant transformades de Wavelet.
    Avançat, però lent. Ideal per I+D offline.
    """
    def __init__(self, wavelet='db4', level=2):
        self.wavelet = wavelet
        self.level = level

    def fit(self, X, y=None):
        return self

    def transform(self, X):
        X_denoised = np.zeros_like(X)
        for i in range(X.shape[0]):
            # Descompon el senyal
            coeffs = pywt.wavedec(X[i, :], self.wavelet, level=self.level)
            # Thresholding a les altes freqüències (posar a 0 els components de soroll)
            sigma = np.median(np.abs(coeffs[-1])) / 0.6745
            uthresh = sigma * np.sqrt(2 * np.log(len(X[i, :])))
            coeffs[1:] = (pywt.threshold(i, value=uthresh, mode='soft') for i in coeffs[1:])
            # Reconstrueix el senyal net
            X_denoised[i, :] = pywt.waverec(coeffs, self.wavelet)
        return X_denoised
    

# =====================================================================
# PAS 2: MOTOR D'AVALUACIÓ DE MÈTRIQUES (AMB OVERFITTING CHECK)
# =====================================================================
def evaluate_pipeline(pipeline, X_train, y_train, X_test, y_test, rejection_threshold=0.85):
    
    # --- A. CHECK D'OVERFITTING (K-FOLD i TRAIN SCORE) ---
    # 1. Nota a l'examen que ja ha vist (Train Score)
    y_train_pred = pipeline.predict(X_train)
    train_f1 = f1_score(y_train, y_train_pred, average='macro')
    
    # 2. K-Fold Cross Validation (5 particions sobre el train)
    cv = StratifiedKFold(n_splits=5, shuffle=True, random_state=42)
    cv_scores = cross_val_score(pipeline, X_train, y_train, cv=cv, scoring='f1_macro')
    cv_mean = cv_scores.mean()
    
    # --- B. MÈTRIQUES DE TEST (Dades no vistes) ---
    start_time = time.perf_counter()
    y_pred = pipeline.predict(X_test)
    inference_time_ms = ((time.perf_counter() - start_time) * 1000) / len(X_test)

    acc = accuracy_score(y_test, y_pred)
    prec = precision_score(y_test, y_pred, average='macro', zero_division=0)
    rec = recall_score(y_test, y_pred, average='macro', zero_division=0)
    test_f1 = f1_score(y_test, y_pred, average='macro')
    kappa = cohen_kappa_score(y_test, y_pred)
    cm = confusion_matrix(y_test, y_pred)

    try:
        y_prob = pipeline.predict_proba(X_test)
        loss = log_loss(y_test, y_prob)
        try:
            auc_ovr = roc_auc_score(y_test, y_prob, multi_class='ovr')
        except ValueError:
            auc_ovr = None 

        max_probs = np.max(y_prob, axis=1)
        rejected_count = np.sum(max_probs < rejection_threshold)
        rejection_rate = rejected_count / len(X_test)
        
    except AttributeError:
        loss, auc_ovr, rejection_rate = None, None, None

    # Càlcul de la "Caiguda" (Drop) per veure l'overfitting d'un cop d'ull
    overfitting_drop = train_f1 - test_f1

    return {
        "Train F1 (Memoritzat)": train_f1,
        "CV F1 (5-Fold Validat)": cv_mean,
        "Test F1 (Real)": test_f1,
        "Overfitting Drop": overfitting_drop,
        "Cohen Kappa": kappa,
        "Accuracy": acc,
        "Precision": prec,
        "Recall": rec,
        "Inference Time (ms)": inference_time_ms,
        "Loss": loss,
        "ROC-AUC (OVR)": auc_ovr,
        "Rejection Rate": rejection_rate,
        "Confusion Matrix": cm
    }

# =====================================================================
# PAS 3: CONFIGURACIÓ DE LA MATRIU DE PIPELINES
# =====================================================================
# --- Configuració Base (TORNEM AL TEU RANDOM FOREST!) ---
# Poso n_estimators=100 com tenies al teu codi original. 
# class_weight='balanced' és vital per no ignorar els materials petits.
clf = RandomForestClassifier(n_estimators=100, class_weight='balanced', random_state=42)

# (Aquí segueix el teu diccionari pipelines_to_test tal com el tenies...)

# --- Adaptador per a PLS-DA ---
class PLSReducer(BaseEstimator, TransformerMixin):
    """Utilitza PLS només per reduir dimensionalitat i passar-ho a la Regressió Logística"""
    def __init__(self, n_components=5):
        self.n_components = n_components
        self.pls = PLSRegression(n_components=n_components)
    def fit(self, X, y):
        # Convertim les etiquetes categòriques a numèriques per al PLS
        y_num = np.unique(y, return_inverse=True)[1]
        self.pls.fit(X, y_num)
        return self
    def transform(self, X):
        return self.pls.transform(X)

# --- Configuració Base ---
clf = LogisticRegression(max_iter=2000, class_weight='balanced')

# NOTA: Per als pipelines amb interpolació, necessitem les longituds d'ona.
# Substitueix wv_floats pels teus 512 valors reals quan integris el codi.
wv_floats = np.linspace(325, 1707, 512) 
wvs_5 = np.concatenate([np.arange(325, 795 + 5, 5), np.arange(990, 1710 + 5, 5)])
wvs_10 = np.concatenate([np.arange(325, 795 + 10, 10), np.arange(990, 1710 + 10, 10)])

# --- DICCIONARI DE COMBINACIONS (Els teus 31 Pipelines) ---
pipelines_to_test = {
    "0. Sense Pre-processament (Baseline)": Pipeline([
        ('clf', clf)  # Cap filtre, directe al Random Forest
    ]),
    
    "1. StandardScaler + SNV": Pipeline([
        ('clamp', ClampingTransformer()), ('snv', SNVTransformer()), 
        ('scaler', StandardScaler()), ('clf', clf)
    ]),
    
    "2. StandardScaler + SNV + Savitzky-Golay": Pipeline([
        ('clamp', ClampingTransformer()), ('sg', SavitzkyGolayTransformer()), 
        ('snv', SNVTransformer()), ('scaler', StandardScaler()), ('clf', clf)
    ]),
    
    "3. StandardScaler + SNV + Savitzky-Golay + 1ra Derivada": Pipeline([
        ('clamp', ClampingTransformer()), ('sg_1deriv', SavitzkyGolayTransformer(deriv=1)), 
        ('snv', SNVTransformer()), ('scaler', StandardScaler()), ('clf', clf)
    ]),
    
    "4. StandardScaler + SNV + Savitzky-Golay + Trimming": Pipeline([
        ('trimming', TrimmingTransformer()), ('clamp', ClampingTransformer()), 
        ('sg', SavitzkyGolayTransformer()), ('snv', SNVTransformer()), 
        ('scaler', StandardScaler()), ('clf', clf)
    ]),
    
    "5. StandardScaler + SNV + Savitzky-Golay + PCA": Pipeline([
        ('clamp', ClampingTransformer()), ('sg', SavitzkyGolayTransformer()), 
        ('snv', SNVTransformer()), ('scaler', StandardScaler()), 
        ('pca', PCA(n_components=5)), ('clf', clf)
    ]),
    
    "6. StandardScaler + SNV + Savitzky-Golay + Interpol. 5": Pipeline([
        ('interp', InterpolationTransformer(wv_floats, wvs_5)), 
        ('sg', SavitzkyGolayTransformer()), ('snv', SNVTransformer()), 
        ('scaler', StandardScaler()), ('clf', clf)
    ]),
    
    "7. StandardScaler + SNV + Savitzky-Golay + Interpol. 10": Pipeline([
        ('interp', InterpolationTransformer(wv_floats, wvs_10)), 
        ('sg', SavitzkyGolayTransformer()), ('snv', SNVTransformer()), 
        ('scaler', StandardScaler()), ('clf', clf)
    ]),
    
    "8. StandardScaler + SNV + Interpol. 5": Pipeline([
        ('interp', InterpolationTransformer(wv_floats, wvs_5)), 
        ('snv', SNVTransformer()), ('scaler', StandardScaler()), ('clf', clf)
    ]),
    
    "9. StandardScaler + SNV + Interpol. 10": Pipeline([
        ('interp', InterpolationTransformer(wv_floats, wvs_10)), 
        ('snv', SNVTransformer()), ('scaler', StandardScaler()), ('clf', clf)
    ]),
    
    "10. StandardScaler + SNV + PCA": Pipeline([
        ('clamp', ClampingTransformer()), ('snv', SNVTransformer()), 
        ('scaler', StandardScaler()), ('pca', PCA(n_components=5)), ('clf', clf)
    ]),
    
    "11. StandardScaler + SNV + Trimming": Pipeline([
        ('trimming', TrimmingTransformer()), ('clamp', ClampingTransformer()), 
        ('snv', SNVTransformer()), ('scaler', StandardScaler()), ('clf', clf)
    ]),
    
    "12. StandardScaler + Trimming": Pipeline([
        ('trimming', TrimmingTransformer()), ('clamp', ClampingTransformer()), 
        ('scaler', StandardScaler()), ('clf', clf)
    ]),
    
    "13. StandardScaler + Trimming + PCA": Pipeline([
        ('trimming', TrimmingTransformer()), ('clamp', ClampingTransformer()), 
        ('scaler', StandardScaler()), ('pca', PCA(n_components=5)), ('clf', clf)
    ]),
    
    "14. StandardScaler + Median Filter + Trimming + PCA": Pipeline([
        ('trimming', TrimmingTransformer()), ('clamp', ClampingTransformer()), 
        ('median', MedianFilterTransformer()), ('scaler', StandardScaler()), 
        ('pca', PCA(n_components=5)), ('clf', clf)
    ]),
    
    "15. StandardScaler + Savitzky-Golay + Trimming + PCA": Pipeline([
        ('trimming', TrimmingTransformer()), ('clamp', ClampingTransformer()), 
        ('sg', SavitzkyGolayTransformer()), ('scaler', StandardScaler()), 
        ('pca', PCA(n_components=5)), ('clf', clf)
    ]),
    
    "16. StandardScaler + Median + SG + Trimming + PCA": Pipeline([
        ('trimming', TrimmingTransformer()), ('clamp', ClampingTransformer()), 
        ('median', MedianFilterTransformer()), ('sg', SavitzkyGolayTransformer()), 
        ('scaler', StandardScaler()), ('pca', PCA(n_components=5)), ('clf', clf)
    ]),
    
    "17. StandardScaler + SNV + Trimming + PCA": Pipeline([
        ('trimming', TrimmingTransformer()), ('clamp', ClampingTransformer()), 
        ('snv', SNVTransformer()), ('scaler', StandardScaler()), 
        ('pca', PCA(n_components=5)), ('clf', clf)
    ]),
    
    "18. StandardScaler + MSC + Trimming + PCA": Pipeline([
        ('trimming', TrimmingTransformer()), ('clamp', ClampingTransformer()), 
        ('msc', MSCTransformer()), ('scaler', StandardScaler()), 
        ('pca', PCA(n_components=5)), ('clf', clf)
    ]),
    
    "19. StandardScaler + SNV + Median + SG + Trimming + PCA": Pipeline([
        ('trimming', TrimmingTransformer()), ('clamp', ClampingTransformer()), 
        ('median', MedianFilterTransformer()), ('sg', SavitzkyGolayTransformer()), 
        ('snv', SNVTransformer()), ('scaler', StandardScaler()), 
        ('pca', PCA(n_components=5)), ('clf', clf)
    ]),
    
    "20. StandardScaler + MSC + Median + SG + Trimming + PCA": Pipeline([
        ('trimming', TrimmingTransformer()), ('clamp', ClampingTransformer()), 
        ('median', MedianFilterTransformer()), ('sg', SavitzkyGolayTransformer()), 
        ('msc', MSCTransformer()), ('scaler', StandardScaler()), 
        ('pca', PCA(n_components=5)), ('clf', clf)
    ]),
    
    "21. StandardScaler + Detrending + Trimming + PCA": Pipeline([
        ('trimming', TrimmingTransformer()), ('clamp', ClampingTransformer()), 
        ('detrending', DetrendingTransformer()), ('scaler', StandardScaler()), 
        ('pca', PCA(n_components=5)), ('clf', clf)
    ]),
    
    "22. StandardScaler + SNV + Detrending + Trimming + PCA": Pipeline([
        ('trimming', TrimmingTransformer()), ('clamp', ClampingTransformer()), 
        ('snv', SNVTransformer()), ('detrending', DetrendingTransformer()), 
        ('scaler', StandardScaler()), ('pca', PCA(n_components=5)), ('clf', clf)
    ]),
    
    "23. StandardScaler + SNV + Median + SG + Detrending + Trimming + PCA": Pipeline([
        ('trimming', TrimmingTransformer()), ('clamp', ClampingTransformer()), 
        ('median', MedianFilterTransformer()), ('sg', SavitzkyGolayTransformer()), 
        ('snv', SNVTransformer()), ('detrending', DetrendingTransformer()), 
        ('scaler', StandardScaler()), ('pca', PCA(n_components=5)), ('clf', clf)
    ]),
    
    "24. StandardScaler + SG + SG 1ra deriv + Trimming + PCA": Pipeline([
        ('trimming', TrimmingTransformer()), ('clamp', ClampingTransformer()), 
        ('sg', SavitzkyGolayTransformer()), ('sg_1deriv', SavitzkyGolayTransformer(deriv=1)), 
        ('scaler', StandardScaler()), ('pca', PCA(n_components=5)), ('clf', clf)
    ]),
    
    "25. StandardScaler + SG + SG 2na deriv + Trimming + PCA": Pipeline([
        ('trimming', TrimmingTransformer()), ('clamp', ClampingTransformer()), 
        ('sg', SavitzkyGolayTransformer()), ('sg_2deriv', SavitzkyGolayTransformer(deriv=2)), 
        ('scaler', StandardScaler()), ('pca', PCA(n_components=5)), ('clf', clf)
    ]),
    
    "26. StandardScaler + SNV + SG + SG 2na deriv + Trimming + PCA": Pipeline([
        ('trimming', TrimmingTransformer()), ('clamp', ClampingTransformer()), 
        ('sg', SavitzkyGolayTransformer()), ('snv', SNVTransformer()), 
        ('sg_2deriv', SavitzkyGolayTransformer(deriv=2)), 
        ('scaler', StandardScaler()), ('pca', PCA(n_components=5)), ('clf', clf)
    ]),
    
    "27. StandardScaler + SNV + Trimming + PLS-DA": Pipeline([
        ('trimming', TrimmingTransformer()), ('clamp', ClampingTransformer()), 
        ('snv', SNVTransformer()), ('scaler', StandardScaler()), 
        ('pls', PLSReducer(n_components=5)), ('clf', clf)
    ]),
    
    "28. StandardScaler + SNV + Median + SG + Detrending + Trimming + PLS-DA": Pipeline([
        ('trimming', TrimmingTransformer()), ('clamp', ClampingTransformer()), 
        ('median', MedianFilterTransformer()), ('sg', SavitzkyGolayTransformer()), 
        ('snv', SNVTransformer()), ('detrending', DetrendingTransformer()), 
        ('scaler', StandardScaler()), ('pls', PLSReducer(n_components=5)), ('clf', clf)
    ]),
    
    "29. StandardScaler + SNV + Wavelet + Detrending + Trimming + PCA": Pipeline([
        ('trimming', TrimmingTransformer()), ('clamp', ClampingTransformer()), 
        ('wavelet', WaveletDenoisingTransformer()), ('snv', SNVTransformer()), 
        ('detrending', DetrendingTransformer()), ('scaler', StandardScaler()), 
        ('pca', PCA(n_components=5)), ('clf', clf)
    ]),
    
    "30. StandardScaler + SNV + SG + Trimming + PCA + SelectKBest": Pipeline([
        ('trimming', TrimmingTransformer()), ('clamp', ClampingTransformer()), 
        ('sg', SavitzkyGolayTransformer()), ('snv', SNVTransformer()), 
        ('select_k', SelectKBest(f_classif, k=100)), ('scaler', StandardScaler()), 
        ('pca', PCA(n_components=5)), ('clf', clf)
    ]),
    
    "31. StandardScaler + SNV + SG + SG 1ra deriv + Trimming + PCA": Pipeline([
        ('trimming', TrimmingTransformer()), ('clamp', ClampingTransformer()), 
        ('sg', SavitzkyGolayTransformer()), ('snv', SNVTransformer()), 
        ('sg_1deriv', SavitzkyGolayTransformer(deriv=1)), 
        ('scaler', StandardScaler()), ('pca', PCA(n_components=5)), ('clf', clf)
    ])
}

# =====================================================================
# PAS 4: BUCLE D'EXECUCIÓ PRINCIPAL I EXPORTACIÓ
# =====================================================================
def load_and_prepare_data(csv_path):
    """
    Llegeix el dataset cru, aplica filtres de qualitat i separa X de y.
    Adaptat a la lògica de negoci del teu script original.
    """
    print(f"Llegint dades des de: {csv_path}...")
    
    # 1. Llegim respectant el format europeu.
    # L'argument decimal=',' substitueix el teu bucle str.replace(',', '.') automàticament.
    df = pd.read_csv(csv_path, sep=';', decimal=',')
    
    # 2. FILTRE DE QUALITAT (La teva aportació clau)
    # Quedar-nos només amb les classes que tenen més de 20 mostres per evitar cracs al stratify
    counts = df['Class'].value_counts()
    classes_bones = counts[counts > 20].index
    df = df[df['Class'].isin(classes_bones)].copy()
    print(f"-> Ens hem quedat amb {len(df)} mostres vàlides de {len(classes_bones)} materials diferents.")
    print(f"   Classes actives: {list(classes_bones)}")
    
    # 3. Separar meta-dades (el text) dels valors purs de la càmera
    meta_cols = ['Measure Type', 'Date', 'Time', 'Counter', 'Label', 'Class']
    wv_cols = [c for c in df.columns if c not in meta_cols]
    
    # Creem X (dades físiques) i y (etiqueta a predir)
    X = df[wv_cols].values.astype(float)
    y = df['Class'].values # Utilitzem Class com al teu codi original
    
    return X, y

def main():
    # 1. Rutes dinàmiques
    directori_actual = os.path.dirname(os.path.abspath(__file__))
    ruta_entrada = os.path.join(directori_actual, 'Dades.csv')
    ruta_sortida_excel = os.path.join(directori_actual, 'Resultats_31_Pipelines.xlsx')

    # 2. Carregar i Netejar Dades
    try:
        X, y = load_and_prepare_data(ruta_entrada)
    except FileNotFoundError:
        print(f"\n[!] ERROR FATAL: No s'ha trobat el fitxer '{ruta_entrada}'.")
        print("Assegura't de posar Dades.csv a la mateixa carpeta que aquest script.")
        return

    # 3. Partició Train/Test (Estratificada, igual que al teu codi)
    print("\nDividint les dades en Entrenament (80%) i Test (20%)...")
    X_train, X_test, y_train, y_test = train_test_split(
        X, y, test_size=0.20, stratify=y, random_state=42
    )
    print(f"-> Entrenant amb {len(X_train)} mostres, testejant amb {len(X_test)} mostres.\n")

    # 4. Execució dels 31 Pipelines
    results_list = []
    
    for name, pipe in pipelines_to_test.items():
        print(f"[{name}] -> Entrenant...")
        try:
            # Entrenem el pipeline complet
            pipe.fit(X_train, y_train)
            
            # Avaluem amb les mètriques i el check d'overfitting (ARA LI PASSEM X_train i y_train!)
            metrics = evaluate_pipeline(pipe, X_train, y_train, X_test, y_test, rejection_threshold=0.85)
            
            cm_str = str(metrics.pop("Confusion Matrix").tolist())
            metrics["Confusion Matrix"] = cm_str
            metrics["Pipeline"] = name
            
            results_list.append(metrics)
            print(f"    ✓ Train F1: {metrics['Train F1 (Memoritzat)']:.4f} | Test F1: {metrics['Test F1 (Real)']:.4f} | Overfitting: -{metrics['Overfitting Drop']:.4f}")
            
        except Exception as e:
            print(f"    [!] ERROR al pipeline: {str(e)}")
            metrics = {
                "Pipeline": name, "Accuracy": None, "Precision": None, "Recall": None,
                "F1-Score": None, "Cohen Kappa": None, "Inference Time (ms)": None,
                "Loss": None, "ROC-AUC (OVR)": None, "Rejection Rate": None, "Confusion Matrix": str(e)
            }
            results_list.append(metrics)

    # 5. Exportació a 2 Excels Diferents
    ruta_original = os.path.join(directori_actual, 'Resultats_31_Pipelines_Original.xlsx')
    ruta_ordenat = os.path.join(directori_actual, 'Resultats_31_Pipelines_Ordenat.xlsx')
    
    print(f"\nGenerant informes d'arquitectura...")
    df_results = pd.DataFrame(results_list)
    
    # Ordenem les columnes de forma lògica per veure l'overfitting ràpidament
    column_order = [
        "Pipeline", "Train F1 (Memoritzat)", "CV F1 (5-Fold Validat)", "Test F1 (Real)", 
        "Overfitting Drop", "Cohen Kappa", "Accuracy", "ROC-AUC (OVR)", 
        "Rejection Rate", "Inference Time (ms)", "Precision", "Recall", "Loss", "Confusion Matrix"
    ]
    df_results = df_results[column_order]
    
    # GUARDAR EXCEL 1: Ordre Original (Tal com s'han executat)
    df_results.to_excel(ruta_original, index=False)
    print(f" -> Guardat: {ruta_original}")
    
    # GUARDAR EXCEL 2: Ordenat per Test F1-Score
    df_results.sort_values(by="Test F1 (Real)", ascending=False, inplace=True)
    df_results.to_excel(ruta_ordenat, index=False)
    print(f" -> Guardat: {ruta_ordenat}")
    
    print("="*50)
    print(" EXPERIMENT FINALITZAT AMB ÈXIT! ")
    print("="*50)

if __name__ == '__main__':
    main()