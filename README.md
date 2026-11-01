# Pneumatic Didactic Library for Hopsan

An independent educational add-on with **24 native components** and **10 runnable circuits** for Hopsan 2.24.4. Assemble pneumatic circuits using ready-made directional valves, actuators, flow and pressure controls, and signal logic.

This is a community prototype, not an official Hopsan release. The public package contains only the add-on and its own examples; course materials and private projects are excluded.

## Components

| Group | Included models |
| --- | --- |
| Directional valves | 3/2 normally closed and normally open; 5/2 spring-return and bistable; 5/3 closed, exhaust and pressure centres |
| Actuators | Double-acting cylinder; spring-return single-acting cylinder |
| Flow and pressure | Restrictor, check valve, one-way flow control, shut-off, regulator, relief valve, quick exhaust |
| Pneumatic logic | Shuttle OR and two-pressure AND |
| Storage and instruments | Air reservoir, pressure gauge/switch, end-position sensors |
| Signal controls | Two-coil command, on-delay timer, teaching sequence |

## Quick start on Windows

1. Install Hopsan **2.24.4, 64 bit** from [the official release](https://github.com/Hopsan/hopsan/releases/tag/v2.24.4).
2. Download this repository as ZIP and extract it to a local path without accented characters.
3. Compile the library with the script below, or use the matching prebuilt `Library/pneumaticadidatica_v04.dll` if included in this release.
4. In Hopsan, choose **Load external library** and select `Library/PneumaticaDidatica.xml` (or its folder).
5. Open `Library/Circuitos/01_53_Centro_Fechado.hmf` and click **Simulate**.
6. Open the signal scopes, or expand an actuator in **Plot Variables** and select its position or pressure output.

The library appears as **PneumaticaDidatica**. Keep its XML, SVG and DLL files together. Pneumatic ports are `NodePneumatic`, not hydraulic ports. Pressures are **absolute Pa** and temperatures are **K**.

## Build and test

The Windows build uses the MinGW compiler shipped with Hopsan 2.24.4:

```powershell
python tools/build.py --hopsan-root "C:/Hopsan/hopsan-2.24.4-win64"
$env:HOPSAN_ROOT = "C:/Hopsan/hopsan-2.24.4-win64"
python -m pip install -r tests/requirements.txt
python tests/validar_valvulas.py
python tests/validar_cilindro.py
python tests/validar_acessorios.py
```

The suites cover valve truth tables and flow reversal, mass/energy balances, bistable memory, cylinder travel, trapped gas, accessories, logic, and timestep sensitivity. Recorded baseline results are in `docs/validation`. Tests generate local CSV/log files, which are ignored by Git. Successful numerical tests do not establish experimental validity.

## Modelling limits

- Ideal gas, quasi-steady compressible valve flow, instantaneous switching and zero leakage in closed valve passages.
- Cylinder mass and energy balances include changing chamber volume and `p*dV`. Mechanical mass, load, spring and damping are internal; there is no external mechanical power port.
- Cylinders and reservoirs use explicit integration and zero pneumatic characteristic impedance. Start with a **50 microsecond** timestep and repeat at half the timestep after changing a circuit.
- A closed-centre valve traps air; it does **not** rigidly lock a moving piston. Chamber pressure can exceed supply pressure.
- The pneumatic logic and regulator are simplified teaching models. No industrial safety certification, complete ISO 1219 conformity, commercial component equivalence, or full FluidSIM equivalence is claimed.
- Linux/macOS builds and other Hopsan versions have not been validated.

See [the Portuguese technical manual](docs/Manual%20em%20portugu%C3%AAs.txt) for port tables, parameters and numerical assumptions. Attribution and methodology are in [AUTORIA.txt](Library/AUTORIA.txt).

## License and references

Apache-2.0; see [LICENSE](LICENSE) and [NOTICE](NOTICE). HopsanCore and the Hopsan default library retain their own original licenses.

- [Hopsan source and documentation](https://github.com/Hopsan/hopsan)
- [Festo MPA-S catalogue, valve functions](https://ftp.festo.com/public/pneumatic/SOFTWARE_SERVICE/Documentation/2017/EN/TYP32_EN.PDF)
- ISO 1219-1, graphical symbols for fluid-power systems. The licensed standard is not distributed in this repository.
