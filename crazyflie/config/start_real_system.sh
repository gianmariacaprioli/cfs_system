#!/usr/bin/env bash

SESSION="realworld"

# Pulizia iniziale
tmux kill-session -t $SESSION 2>/dev/null
tmux new-session -d -s $SESSION
#tmux new-session -d -x "$(tput cols)" -y "$(tput lines)" -s $SESSION

# ==========================================
# FASE 1: CREAZIONE DELLA GEOMETRIA (Zero Comandi)
# ==========================================
# Ottengo l'ID del primissimo pannello (Alto Sx)
P_ALTO_SX=$(tmux display-message -p -t $SESSION '#{pane_id}')

# 1. Taglio a metà lo schermo verticalmente (creo la colonna di Destra)
P_ALTO_DX=$(tmux split-window -h -t $P_ALTO_SX -P -F '#{pane_id}')

# 2. Taglio a metà la Destra (creo Basso Dx)
P_BASSO_DX=$(tmux split-window -v -t $P_ALTO_DX -P -F '#{pane_id}')

# 3. Taglio a metà la Sinistra (creo Basso Sx)
P_BASSO_SX_SOPRA=$(tmux split-window -v -t $P_ALTO_SX -P -F '#{pane_id}')

# 4. Taglio Basso Sx orizzontalmente (creo la riga sotto del Basso Sx)
P_BASSO_SX_SOTTO_SX=$(tmux split-window -v -t $P_BASSO_SX_SOPRA -P -F '#{pane_id}')

# 5. Taglio quest'ultima riga a metà verticalmente (creo la colonna di destra dei topic)
P_BASSO_SX_SOTTO_DX=$(tmux split-window -h -t $P_BASSO_SX_SOTTO_SX -P -F '#{pane_id}')

# ==========================================
# FASE 2: ASSEGNAZIONE DEI COMANDI
# Ora che l'architettura è stabile, spariamo i comandi ai bersagli esatti
# ==========================================

# 1. ALTO SINISTRA (Gazebo)
tmux send-keys -t $P_ALTO_SX "ros2 launch crazyflie_examples StartSystem_real.py" C-m

# 2. ALTO DESTRA (Backend)
tmux send-keys -t $P_ALTO_DX "ros2 run crazyflie_examples nice_hover"
# 3. BASSO SINISTRA - SOPRA (Root Real)
tmux send-keys -t $P_BASSO_SX_SOPRA "ros2 run crazyflie_examples cfs_root_real" 

# 4. BASSO SINISTRA - COLONNA SX (Echo Pose)
tmux send-keys -t $P_BASSO_SX_SOTTO_SX "ros2 topic echo /cf_0/pose" 

# 5. BASSO SINISTRA - COLONNA DX (Echo Cmd Vel)
tmux send-keys -t $P_BASSO_SX_SOTTO_DX "ros2 run python_parameters fig" 

# 6. BASSO DESTRA (Python Parameters) - Senza C-m per lasciarlo in attesa
tmux send-keys -t $P_BASSO_DX "ros2 launch python_parameters crazy.launch.py"

# ==========================================
# FINALIZZAZIONE
# ==========================================
# Sposta il cursore sul pannello in basso a destra per farti lanciare i parametri
# tmux select-pane -t $P_BASSO_DX
tmux attach -t $SESSION