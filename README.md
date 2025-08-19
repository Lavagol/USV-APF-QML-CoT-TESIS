# USV-APF-QML-CoT-TESIS
Simulador de un USV (vehículo de superficie no tripulado) con **planificador local APF**, **GUI en QML** y manejo de **coordenadas reales** (WGS84→UTM→XY interno). El sistema corre en dos procesos: **Servidor** (datos) y **Cliente** (interfaz QML / simulador / recomendación).

---

## ¿Qué incluye?
- **Servidor** (`server/utils`): **emite** la posición inicial del USV, el punto final (meta) y la lista de obstáculos **en coordenadas GPS** vía socket.  
- **Cliente** (`client`): **recibe** los datos del servidor, realiza las **conversiones WGS84→UTM→XY interno**, ejecuta el **planificador APF** (incluye modos preventivo y escape) y presenta la **GUI en QML**.
- **Distancias reales**: simulación en metros, con radios de alerta/activación configurables en el cliente.
- **Logs**: el **Cliente** muestra por consola las conversiones y fuerzas del APF; el **Servidor** muestra estado de escucha y conexiones.

---

## Requisitos
- **Python 3.10 – 3.13**
- Git (opcional, para clonar)
- Sistema operativo: **Windows**, **macOS**, **Linux** o **Raspberry Pi**
---

## Instalación 

### Windows (PowerShell)
```powershell
cd C:\ruta\a\TESIS_GIT
py -3 -m venv .venv
. .\.venv\Scripts\Activate.ps1
pip install -r requirements.txt
```

### macOS / Linux / Raspberry Pi (bash)
```bash
cd ~/TESIS_GIT
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

> Si aparece un error de permisos en Windows, ejecuta PowerShell como **Administrador** o cambia la política de ejecución:
> ```powershell
> Set-ExecutionPolicy -Scope CurrentUser RemoteSigned
> ```

---

## Ejecución rápida (cada vez que quieras usar el simulador)

Se necesitan **dos terminales**: una para el **Servidor** y otra para el **Cliente**.

### 1) Iniciar el **Servidor** (Terminal A)
Debe ejecutarse desde `server/utils`.

**Windows**
```powershell
cd C:\ruta\a\TESIS_GIT
. .\.venv\Scripts\Activate.ps1
cd server\utils
python main.py
```

**macOS / Linux / RPi**
```bash
cd ~/TESIS_GIT
source .venv/bin/activate
cd server/utils
python3 main.py
```

**Debe aparecer:**  
`Servidor escuchando en 0.0.0.0:65432`

---

### 2) Iniciar el **Cliente / GUI QML** (Terminal B)
Ejecutar como **módulo** desde la raíz del repo.

**Windows**
```powershell
cd C:\ruta\a\TESIS_GIT
. .\.venv\Scripts\Activate.ps1
python -m client.main_gui
```

**macOS / Linux / RPi**
```bash
cd ~/TESIS_GIT
source .venv/bin/activate
python3 -m client.main_gui
```

**Debe aparecer:**
- `Conexión establecida con el servidor.`
- Logs de conversión **WGS84 → UTM** y fuerzas **APF**.

---

### 3) Detener
En cada terminal, presiona **Ctrl + C**.

---

### 4) Consideraciones y visualización

- **Archivos .npy (trayectoria y obstáculos):**  
  En cada simulación se guardan arrays **.npy** con la **trayectoria** y los **obstáculos**. Para visualizar el comportamiento del USV a partir de estos archivos, ejecuta:
  ```bash
  # Desde la raíz del repo
  python graficador.py    # o 'python3 graficador.py'
  ```

- **Fuerzas atractivas/repulsivas (runXXXXX.csv):**  
  Para verificar el comportamiento de las fuerzas, carga el archivo **runXXX.csv** en:
  ```bash
  python graficofuerzas.py
  ```
  Dentro del script podrás seleccionar el **runXXX.csv** generado por la simulación.

- **Archivos .json (para mapa real):**  
  Cada simulación genera **`trayectoria_gps.json`** y **`obstaculos_gps.json`**.  
  > **Importante:** al iniciar una nueva simulación, los **.json anteriores se sobrescriben**.  
  Estos **.json** sirven para visualizar el recorrido en un mapa real mediante:
  ```bash
  python generar_mapas_multiples.py
  ```
  (El script toma los JSON actuales para crear el mapa.)

- **Persistencia de simulaciones:**  
  Las simulaciones se **guardan** solo si el **USV alcanza la meta** (condición de éxito). Si no llega, no se genera/sobrescribe el set completo de resultados.

---

## Estructura del proyecto
```text
TESIS_GIT/
├─ client/
│  ├─ APF/
│  │  └─ recomendacion.py
│  ├─ simulador/
│  │  └─ simulador.py
│  ├─ handlers/
│  │  └─ socket_handlercorreo.py
│  ├─ models/
│  │  ├─ obstaculo.py
│  │  └─ parametros_obstaculos.py
│  ├─ ui/
│  │  └─ interface.qml
│  ├─ utils/
│  │  └─ graficador.py
│  ├─ main_gui.py          # Punto de entrada GUI (ejecutar como módulo)
│  └─ debug_cli.py         # Punto de entrada consola (opcional)
├─ server/
│  └─ utils/
│     ├─ main.py           # Punto de entrada del server (ejecutar desde esta carpeta)
│     ├─ handlers/
│     │  ├─ nmea_handler.py
│     │  └─ socket_handler.py
│     └─ utils/
│        ├─ servidor_ip.py
│        └─ generar_cot.py
├─ graficofuerzas.py
├─ generar_mapas_multiples.py
├─ requirements.txt
└─ README.md
```

---

## Parámetros y configuración
- **Puerto/host**: definidos en el servidor (por defecto `0.0.0.0:65432`).
- **Entradas** (inicio / obstáculos / meta): el servidor las consume desde las fuentes configuradas en `server/utils/main.py`.
- **Planificador APF**: constantes variables `k_att` (`recomendacion.py`), `k_rep` y `d0` (`parametros_obstaculos.py`).  
  Los demás parámetros se dejarán constantes respecto a la tabla del trabajo (Tabla 5.9: Configuraciones recomendadas por tipo de obstáculo y velocidad).
- Todos los parámetros pueden ser variados según el uso, pero es importante mantener equilibrio entre ellos, en especial los que participan en el planificador local APF. 

---

## Prueba rápida 
1. Iniciar **Servidor** (Terminal A) → verifica “servidor escuchando…”.  
2. Iniciar **Cliente** (Terminal B) → verifica “Conexión establecida…”.  
3. Observa logs de WGS84→UTM y fuerzas APF.  

---


## IMPORTANTE

**¿Debo abrir siempre dos terminales?**  
Sí: una para el **Servidor** y otra para el **Cliente**.

**¿Dónde veo las fuerzas APF y conversiones?**  
En la **consola del Cliente** (el Servidor solo muestra estado y conexiones).

