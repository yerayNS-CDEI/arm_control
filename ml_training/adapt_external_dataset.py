import pandas as pd
import numpy as np
from scipy.interpolate import interp1d

def get_target_wavelengths(template_csv):
    """
    Extreu les freqüències exactes del teu dataset original per evitar errors de precisió.
    """
    df_template = pd.read_csv(template_csv, sep=';', nrows=0)
    wavelength_str_cols = df_template.columns[6:] 
    target_wavelengths = np.array([float(col.replace(',', '.')) for col in wavelength_str_cols])
    return target_wavelengths, wavelength_str_cols

def adapt_external_dataset(input_csv, template_csv, output_csv):
    """
    input_csv: Ruta del dataset extern (ex: Dataset_benjamin.csv).
    template_csv: Ruta del teu Dades.csv original (per copiar l'estructura).
    output_csv: Ruta del nou dataset adaptat en format CSV compatibles.
    """
    
    # 1. Obtenir les 512 bandes exactes de la nostra càmera com a referència
    target_wavelengths_float, target_wavelengths_str = get_target_wavelengths(template_csv)
    
    print(f"Llegint {input_csv}...")
    df_external = pd.read_csv(input_csv, sep=',', engine='python')
    
    # 2. Adaptació de noms de columnes al teu estàndard
    if 'material' in df_external.columns and 'description' in df_external.columns:
        df_external.rename(columns={'material': 'Label', 'description': 'Class'}, inplace=True)
    
    # Descartar la classe 'exclude' (ignorant majúscules/minúscules)
    df_external = df_external[df_external['Label'].astype(str).str.lower() != 'exclude'].copy()
    
    # =================================================================================
    # MILLORA DE LABELS: Format "Title Case" i neteja d'espais
    # Ex: "white paint" o " WHITE PAINT " es converteix en "White Paint"
    # =================================================================================
    df_external['Label'] = df_external['Label'].astype(str).str.strip().str.title()
    
    traduccio_classes = {
        "Cellulose_Wood_Npv": "Wood",
        "Plaster": "Plasterboard",
        "Plastic": "Resin", # (Opcional, si creus químicament que es comporten igual al NIR)
    }
    df_external['Label'] = df_external['Label'].replace(traduccio_classes)
    
    wavelength_cols = [col for col in df_external.columns if col.replace('.', '', 1).isdigit()]
    source_wavelengths = np.array([float(col) for col in wavelength_cols])
    
    # =================================================================================
    # 3. NETEJA INTEL·LIGENT (Tolerància a dades faltants)
    # =================================================================================
    # Convertim qualsevol espai/lletra a 'NaN'
    df_external[wavelength_cols] = df_external[wavelength_cols].apply(pd.to_numeric, errors='coerce')
    
    mida_original = len(df_external)
    
    # Calculem quin és el 70% de les columnes espectrals
    min_valid_cols = int(len(wavelength_cols) * 0.70)
    
    # thresh: Requereix que la fila tingui ALMENYS aquest número de valors NO-NaN per sobreviure
    df_external = df_external.dropna(subset=wavelength_cols, thresh=min_valid_cols).copy()
    mida_neta = len(df_external)
    
    print(f"S'han descartat només {mida_original - mida_neta} files per estar massa corrompudes (<70% de dades vàlides).")
    
    labels = df_external['Label'].values
    spectra_raw = df_external[wavelength_cols].values
    
    # 4. Lògica d'escalat
    if np.max(spectra_raw) > 2.0:
        spectra_raw = spectra_raw / 100.0
    
    adapted_spectra = []
    valid_labels = []
    
    # 5. Interpolació mostra per mostra
    adapted_spectra = []
    valid_labels = []
    mostres_fora_de_rang = 0  # <--- COMPTADOR NOU
    
    # 5. Interpolació mostra per mostra
    for i, spectrum in enumerate(spectra_raw):
        valid_idx = ~np.isnan(spectrum)
        x_valid = source_wavelengths[valid_idx]
        y_valid = spectrum[valid_idx]
        
        # Comprovem si té la cobertura de nanòmetres necessària
        if (len(x_valid) > 1 and 
            np.min(x_valid) <= np.min(target_wavelengths_float) and 
            np.max(x_valid) >= np.max(target_wavelengths_float)):
            
            f_interp = interp1d(x_valid, y_valid, kind='linear', assume_sorted=False)
            new_spectrum = f_interp(target_wavelengths_float)
            
            adapted_spectra.append(new_spectrum)
            valid_labels.append(labels[i])
        else:
            mostres_fora_de_rang += 1  # <--- COMPTEM LES QUE NO ARRIBEN AL NIR O VIS
            
    # 6. Generar DataFrame final...
    # (El codi segueix igual...)
            
    # 6. Generar DataFrame final
    df_adapted = pd.DataFrame(adapted_spectra, columns=target_wavelengths_str)
    df_adapted.insert(0, 'Label', valid_labels)
    df_adapted.insert(1, 'Class', df_adapted['Label'].apply(lambda x: str(x)[0] if pd.notnull(x) else ''))
    df_adapted.insert(0, 'Measure Type', 'External')
    df_adapted.insert(1, 'Date', '01/01/2026')
    df_adapted.insert(2, 'Time', '00:00:00')
    df_adapted.insert(3, 'Counter', 0)
    
    # 7. Exportar
    for col in target_wavelengths_str:
        df_adapted[col] = df_adapted[col].apply(lambda x: f"{x:.6f}".replace('.', ','))
        
    df_adapted.to_csv(output_csv, sep=';', index=False)
    
    print(f"Dataset adaptat amb èxit!")
    print(f"Mostres retingudes i interpolades finalment: {len(df_adapted)}")
    print(f"Fitxer desat a: {output_csv}")
    print(f"S'han descartat {mostres_fora_de_rang} mostres addicionals per no tenir cobertura entre {np.min(target_wavelengths_float):.1f}nm i {np.max(target_wavelengths_float):.1f}nm.")
    print(f"Mostres retingudes i interpolades finalment: {len(df_adapted)}")

if __name__ == "__main__":
    adapt_external_dataset(
        input_csv='Dataset_benjamin.csv',
        template_csv='Dades_interp_5.csv',
        output_csv='Nou_Dataset_Adaptat_5.csv'
    )