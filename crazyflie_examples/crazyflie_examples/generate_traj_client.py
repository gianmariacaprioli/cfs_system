#!/usr/bin/env python3
import argparse
import requests
import sys
import os

# Configurazione del server (Container A)
# Dato che sei in --network host, localhost punta al tuo PC host
SERVER_URL = "http://127.0.0.1:5000/generate"

# SERVER_URL = "https://leontine-biosocial-adeline.ngrok-free/generate"
def main():
    # 1. Configurazione degli argomenti da riga di comando
    parser = argparse.ArgumentParser(description="Client per generare traiettorie Crazyswarm via Container Remoto")
    
    parser.add_argument("-i", "--input", required=True, help="Percorso del file CSV con i waypoints (Input)")
    parser.add_argument("-o", "--output", required=True, help="Percorso dove salvare il CSV della traiettoria (Output)")
    parser.add_argument("--v_max", type=float, default=1.0, help="Velocità massima (default: 1.0)")
    parser.add_argument("--a_max", type=float, default=1.0, help="Accelerazione massima (default: 1.0)")
    
    args = parser.parse_args()

    # 2. Verifica esistenza file input
    if not os.path.exists(args.input):
        print(f"ERRORE: Il file di input '{args.input}' non esiste.", file=sys.stderr)
        sys.exit(1)

    print(f"--> Leggendo {args.input}...")
    try:
        with open(args.input, 'r') as f:
            csv_content = f.read()
    except Exception as e:
        print(f"ERRORE nella lettura del file: {e}", file=sys.stderr)
        sys.exit(1)

    # 3. Preparazione richiesta
    payload = {
        "csv_content": csv_content,
        "v_max": args.v_max,
        "a_max": args.a_max
    }

    # 4. Invio richiesta al server
    print(f"--> Inviando richiesta a {SERVER_URL} (v_max={args.v_max}, a_max={args.a_max})...")
    try:
        response = requests.post(SERVER_URL, json=payload, timeout=10)
    except requests.exceptions.ConnectionError:
        print("\nERRORE DI CONNESSIONE:", file=sys.stderr)
        print("Non riesco a contattare il server delle traiettorie su localhost:5000.", file=sys.stderr)
        print("Assicurati che il container 'trajserver' sia avviato con '-p 5000:5000'.", file=sys.stderr)
        sys.exit(1)
    except Exception as e:
        print(f"ERRORE generico: {e}", file=sys.stderr)
        sys.exit(1)

    # 5. Gestione risposta
    if response.status_code == 200:
        try:
            data = response.json()
            traj_csv = data.get("trajectory_csv")
            
            if not traj_csv:
                print("ERRORE: Il server ha risposto OK ma il campo 'trajectory_csv' è vuoto.", file=sys.stderr)
                sys.exit(1)

            # Scrittura output
            with open(args.output, 'w') as f:
                f.write(traj_csv)
            
            print(f"--> SUCCESSO! Traiettoria salvata in: {args.output}")
            sys.exit(0)

        except ValueError:
            print("ERRORE: Risposta non valida dal server (non è un JSON).", file=sys.stderr)
            sys.exit(1)
    else:
        print(f"ERRORE DAL SERVER ({response.status_code}):", file=sys.stderr)
        print(response.text, file=sys.stderr)
        sys.exit(1)

if __name__ == "__main__":
    main()