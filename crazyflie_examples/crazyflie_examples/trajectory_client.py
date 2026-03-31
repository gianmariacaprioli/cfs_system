import requests
import os

# URL di default
DEFAULT_SERVER_URL = "http://127.0.0.1:5000/generate"

def generate_trajectory(input_path, output_path=None, v_max=0.3, a_max=0.3, server_url=DEFAULT_SERVER_URL):
    """
    Invia un CSV di waypoint a un server remoto per generare una traiettoria.

    Args:
        input_path (str): Percorso del file CSV con i waypoint.
        output_path (str, optional): Percorso dove salvare il CSV generato. 
                                     Se None, la funzione restituisce il contenuto CSV come stringa.
        v_max (float): Velocità massima.
        a_max (float): Accelerazione massima.
        server_url (str): URL del server di generazione traiettorie.

    Returns:
        str: Il contenuto del CSV generato (se output_path è None).
        str: Il percorso del file salvato (se output_path è specificato).

    Raises:
        FileNotFoundError: Se il file di input non esiste.
        ConnectionError: Se non è possibile contattare il server.
        ValueError: Se il server risponde con errore o dati non validi.
    """

    # 1. Verifica esistenza file input
    if not os.path.exists(input_path):
        raise FileNotFoundError(f"Il file di input '{input_path}' non esiste.")

    try:
        with open(input_path, 'r') as f:
            csv_content = f.read()
    except Exception as e:
        raise IOError(f"Errore nella lettura del file: {e}")

    # 2. Preparazione richiesta
    payload = {
        "csv_content": csv_content,
        "v_max": v_max,
        "a_max": a_max
    }

    # 3. Invio richiesta al server
    try:
        response = requests.post(server_url, json=payload, timeout=10)
    except requests.exceptions.ConnectionError:
        error_msg = (
            f"Impossibile connettersi al server: {server_url}\n"
            "Assicurati che il container 'trajserver' sia avviato."
        )
        raise ConnectionError(error_msg)
    except Exception as e:
        raise ConnectionError(f"Errore generico di connessione: {e}")

    # 4. Gestione risposta
    if response.status_code == 200:
        try:
            data = response.json()
            traj_csv = data.get("trajectory_csv")
            
            if not traj_csv:
                raise ValueError("Il server ha risposto OK ma il campo 'trajectory_csv' è vuoto.")

            # Se è stato specificato un percorso di output, salva il file
            if output_path:
                with open(output_path, 'w') as f:
                    f.write(traj_csv)
                return output_path
            
            # Altrimenti restituisce il contenuto grezzo
            return traj_csv

        except ValueError as e:
            raise ValueError(f"Errore nel parsing della risposta server: {e}")
    else:
        raise ValueError(f"Errore dal server ({response.status_code}): {response.text}")

# Esempio di utilizzo se lanciato come script standalone (retrocompatibilità)
if __name__ == "__main__":
    import argparse
    import sys

    parser = argparse.ArgumentParser(description="Client per generare traiettorie Crazyswarm")
    parser.add_argument("-i", "--input", required=True, help="File CSV Input")
    parser.add_argument("-o", "--output", required=True, help="File CSV Output")
    parser.add_argument("--v_max", type=float, default=0.5)
    parser.add_argument("--a_max", type=float, default=0.5)
    
    args = parser.parse_args()

    try:
        result = generate_trajectory(
            input_path=args.input,
            output_path=args.output,
            v_max=args.v_max,
            a_max=args.a_max
        )
        print(f"--> SUCCESSO! Traiettoria salvata in: {result}")
    except Exception as e:
        print(f"ERRORE: {e}", file=sys.stderr)
        sys.exit(1)