import pandas as pd
import numpy as np
import time
from sklearn.metrics import (f1_score, accuracy_score, precision_score, 
                             recall_score, cohen_kappa_score, confusion_matrix, 
                             log_loss, roc_auc_score)
from sklearn.model_selection import StratifiedKFold, cross_val_score
from train_final_model import pipeline_produccio 

# =====================================================================
# 1. FUNCIÓ D'AVALUACIÓ COMPLETA (Amb Totes les Mètriques)
# =====================================================================
def evaluate_pipeline(pipeline, X_train, y_train, X_test, y_test, rejection_threshold=0.85):
    # --- A. CHECK D'OVERFITTING (K-FOLD i TRAIN SCORE) ---
    y_train_pred = pipeline.predict(X_train)
    train_f1 = f1_score(y_train, y_train_pred, average='macro')
    
    cv = StratifiedKFold(n_splits=5, shuffle=True, random_state=42)
    cv_scores = cross_val_score(pipeline, X_train, y_train, cv=cv, scoring='f1_macro')
    cv_mean = cv_scores.mean()
    
    # --- B. MÈTRIQUES DE TEST (Dades de Benjamín) ---
    start_time = time.perf_counter()
    y_pred = pipeline.predict(X_test)
    inference_time_ms = ((time.perf_counter() - start_time) * 1000) / len(X_test)

    acc = accuracy_score(y_test, y_pred)
    prec = precision_score(y_test, y_pred, average='macro', zero_division=0)
    rec = recall_score(y_test, y_pred, average='macro', zero_division=0)
    test_f1 = f1_score(y_test, y_pred, average='macro')
    kappa = cohen_kappa_score(y_test, y_pred)
    
    # Matriu de Confusió intel·ligent amb Pandas per evitar errors de forma
    labels_comunes = np.unique(np.concatenate((y_test, y_pred)))
    cm = confusion_matrix(y_test, y_pred, labels=labels_comunes)
    cm_df = pd.DataFrame(cm, index=labels_comunes, columns=labels_comunes)

    try:
        y_prob = pipeline.predict_proba(X_test)
        
        # Protecció matemàtica: si falten classes al test, Loss o AUC poden fallar
        try:
            loss = log_loss(y_test, y_prob, labels=pipeline.classes_)
        except ValueError:
            loss = None
            
        try:
            auc_ovr = roc_auc_score(y_test, y_prob, multi_class='ovr', labels=pipeline.classes_)
        except ValueError:
            auc_ovr = None 

        max_probs = np.max(y_prob, axis=1)
        rejected_count = np.sum(max_probs < rejection_threshold)
        rejection_rate = rejected_count / len(X_test)
        
    except AttributeError:
        loss, auc_ovr, rejection_rate = None, None, None

    # Càlcul de la "Caiguda" per veure l'overfitting d'un cop d'ull
    overfitting_drop = train_f1 - test_f1

    # Retornem TOTES les mètriques sol·licitades
    return {
        "Train F1 (Memoritzat)": train_f1,
        "CV F1 (5-Fold Validat)": cv_mean,
        "Test F1 (Real Benjamín)": test_f1,
        "Overfitting Drop": overfitting_drop,
        "Cohen Kappa": kappa,
        "Accuracy": acc,
        "Precision": prec,
        "Recall": rec,
        "Inference Time (ms)": inference_time_ms,
        "Loss": loss,
        "ROC-AUC (OVR)": auc_ovr,
        "Rejection Rate": rejection_rate,
        "Confusion Matrix": cm_df
    }

def print_metrics(metrics_dict):
    for key, value in metrics_dict.items():
        if key == "Confusion Matrix":
            print(f"\n{key} (Files=Realitat, Columnes=Predicció):\n{value}")
        elif value is None:
            print(f"{key}: N/A (Falten classes al dataset de test per calcular-ho)")
        else:
            print(f"{key}: {value:.4f}")

# =====================================================================
# 2. LÒGICA DE CÀRREGA DE DADES
# =====================================================================
def load_dataset(file_path):
    df = pd.read_csv(file_path, sep=';', decimal=',')
    freq_cols = [col for col in df.columns if col.replace(',', '.', 1).replace('.', '', 1).isdigit()]
    return df[freq_cols].values, df['Label'].values

# =====================================================================
# 3. EXECUCIÓ DEL TEST UNIDIRECCIONAL
# =====================================================================
# =====================================================================
# 3. EXECUCIÓ DEL TEST UNIDIRECCIONAL (Dins de cross_validation_test.py)
# =====================================================================
def run_cross_test():
    print("="*70)
    print(" INICIANT TESTING UNIDIRECCIONAL: TRAIN NADIU -> TEST BENJAMÍN")
    print("="*70)
    
    # ++++ CANVIA AQUESTS NOMS ++++
    X_native, y_native = load_dataset('Dades_interp_5.csv')
    X_benjamin, y_benjamin = load_dataset('Nou_Dataset_Adaptat_5.csv')
    # +++++++++++++++++++++++++++++
    classes_comunes = list(set(y_native).intersection(set(y_benjamin)))
    
    if len(classes_comunes) < 2:
        print("❌ ERROR: Necessitem almenys 2 classes en comú per classificar.")
        return
        
    classes_valides = []
    for cl in classes_comunes:
        # Exigim >= 5 mostres NOMÉS per al teu dataset (per poder entrenar).
        # Les dades d'en Benjamín entren a test, encara que només tinguin 1 mostra.
        if np.sum(y_native == cl) >= 5:
            classes_valides.append(cl)
        else:
            print(f"ATENCIÓ: La classe '{cl}' té menys de 5 mostres al dataset Nadiu. S'exclou.")

    if len(classes_valides) < 2:
        print("❌ ERROR: No han quedat suficients classes vàlides per entrenar.")
        return

    print(f"-> Classes compartides que s'avaluaran: {classes_valides}\n")

    # Filtrem
    idx_nat = np.isin(y_native, classes_valides)
    X_nat_c, y_nat_c = X_native[idx_nat], y_native[idx_nat]
    
    idx_ben = np.isin(y_benjamin, classes_valides)
    X_ben_c, y_ben_c = X_benjamin[idx_ben], y_benjamin[idx_ben]

    print(f"Volum d'Entrenament (El teu robot): {len(y_nat_c)} mostres")
    print(f"Volum de Test (Dades d'en Benjamín): {len(y_ben_c)} mostres\n")
    
    print("--- PROVA ÚNICA: Entrenant model de producció i Avaluant l'extern ---")
    pipeline_produccio.fit(X_nat_c, y_nat_c)
    metrics = evaluate_pipeline(pipeline_produccio, X_nat_c, y_nat_c, X_ben_c, y_ben_c)
    print_metrics(metrics)
    print("="*70)

if __name__ == '__main__':
    run_cross_test()