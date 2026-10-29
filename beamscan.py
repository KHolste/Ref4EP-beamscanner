"""
Beamscan: Steuerung der Linearachse (z, im Vakuum) ueber eine NI PCIe-6321.

Hardware:
  * ao0  -> Relais "HOCH"   (Motor faehrt nach oben)
  * ao1  -> Relais "RUNTER" (Motor faehrt nach unten)
    Die Relais schalten bei ca. 1,3 V, die Ausgaenge duerfen hoechstens
    mit 2,0 V belastet werden.
  * ai0  -> Positionspoti (differentiell gemessen, ai0+/ai8 als ai0-)
    unterster Punkt ca. -4,5 V (Motor stoppt dort selbst),
    oberster Punkt ca. +4,9 V, Hub dazwischen 92 mm.

Sicherheitskonzept:
  * Beim Start wird NICHTS an die Hardware geschrieben.
  * "Verbinden" reserviert das AO-Subsystem und setzt beide Kanaele auf 0 V.
  * Ein Kanal gibt seinen Sollwert erst aus, wenn er freigeschaltet ist.
  * Sollwerte sind auf 0 ... 2,0 V begrenzt (GUI, Software und DAQmx-Task).
  * ai0 wird fest differentiell im Bereich +/-10 V gemessen.
  * HOCH und RUNTER koennen nie gleichzeitig aktiv sein (Verriegelung in
    der GUI und zusaetzlich direkt vor dem Schreiben auf die Hardware).
  * Bei laufender Positionsmessung wird HOCH an der oberen und RUNTER an
    der unteren Grenze automatisch abgeschaltet. Ohne Messung gibt es oben
    keinen automatischen Stopp ("Ganz nach oben" braucht daher die Messung).
  * NOT-AUS sowie Trennen/Programmende setzen beide Ausgaenge auf 0 V.

Start:  python beamscan.py
"""

from __future__ import annotations

import json
import os
import sys
import time
import traceback
from collections import deque

from PyQt6.QtCore import Qt, QTimer, pyqtSignal
from PyQt6.QtGui import QFont
from PyQt6.QtWidgets import (
    QApplication, QCheckBox, QComboBox, QDoubleSpinBox, QFileDialog,
    QGridLayout,
    QGroupBox, QHBoxLayout, QInputDialog, QLabel, QMessageBox,
    QPlainTextEdit,
    QPushButton, QSpinBox, QVBoxLayout, QWidget,
)

try:
    import nidaqmx
    from nidaqmx.system import System
    from nidaqmx.constants import AcquisitionType, TerminalConfiguration
except ImportError:                                     # pragma: no cover
    nidaqmx = None
    System = None
    AcquisitionType = None
    TerminalConfiguration = None

try:
    import pyqtgraph as pg
except ImportError:                                     # pragma: no cover
    pg = None

# --- Analogausgaenge (Relais) ---------------------------------------------
AO_MIN = 0.0            # negative Spannungen werden nicht gebraucht
AO_MAX = 2.0            # maximale Belastung der Relaiseingaenge
RELAIS_SCHWELLE = 1.3   # ab hier zieht das Relais (ungefaehr)
SOLL_STANDARD = 1.3     # Standard-Sollwert fuer HOCH und RUNTER
KANAL_HOCH = 0          # ao0
KANAL_RUNTER = 1        # ao1

# --- Analogeingang (Positionspoti) ----------------------------------------
AI_KANAL = "ai0"
AI_BEREICH = 10.0                       # V, fester Messbereich +/-10 V (Poti -4,5...+4,9 V)
AI_ABTASTRATE = 10000.0                 # Hz, nur bei Mittelung > 1 Sample
# Zeitfenster der Trendkurve (Anzeige, Sekunden)
SEIT_MESSBEGINN = 0         # Zeitfenster-Wert: alles seit "Messung starten"
ZEITFENSTER = [("1 min", 60), ("5 min", 300), ("15 min", 900),
               ("30 min", 1800), ("1 h", 3600),
               ("seit Beginn der Messung", SEIT_MESSBEGINN)]
VERLAUF_FEIN_S = 3600       # jeder Messpunkt wird 1 h lang gespeichert
VERLAUF_GROB_S = 7 * 86400  # ausgeduennter Verlauf, max. 7 Tage
VERLAUF_GROB_RASTER = 10.0  # s, ein Punkt je 10 s im groben Verlauf
Z_ANSICHT = (-2.0, 94.0)    # mm, Standard-Ausschnitt der z-Achse (ganzer Hub)

# Kalibrierung Poti: Spannung am unteren / oberen Anschlag und Hub dazwischen
POTI_U_UNTEN = -4.5     # V, unterster Punkt (Motor stoppt hier selbst)
POTI_U_OBEN = 4.9       # V, oberster Punkt
HUB_MM = 92.0           # mm zwischen den beiden Punkten

GRENZE_UNTEN_STANDARD = 0.5     # mm, RUNTER wird hier abgeschaltet
GRENZE_OBEN_STANDARD = 91.0     # mm, HOCH wird hier abgeschaltet

# Position anfahren
ZIEL_TOLERANZ = 0.05    # mm, so nah am Ziel gilt "schon da"
NACHLAUF_S = 1.0        # s nach dem Abschalten warten, bis die Achse steht
VORHALT_MAX = 5.0       # mm, Obergrenze fuer den gelernten Nachlauf
POSITIONEN_DATEI = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "beamscan_positionen.json")

# Datenspeicherung (nur auf Wunsch): Abstand zwischen zwei gespeicherten Zeilen
SPEICHER_INTERVALLE = [("bei jeder Messung", 0), ("jede Sekunde", 1),
                       ("alle 10 Sekunden", 10), ("jede Minute", 60)]
SPEICHER_INTERVALL_STANDARD = 1         # Index in SPEICHER_INTERVALLE -> 1 s


def zustand_stil(aktiv: bool, typ: str = "QLabel") -> str:
    """Einheitliche Zustandsfarben: hellgruen = aktiv, hellorange = aus."""
    if aktiv:
        hinter, rand, hover = "#b9f6ca", "#2e7d32", "#a3f0ba"
    else:
        hinter, rand, hover = "#ffd59e", "#e67e22", "#ffc97a"
    stil = ("{0} {{ background: {1}; border: 1px solid {2}; border-radius: 4px;"
            " padding: 3px 10px; color: #222222; font-weight: bold; }}"
            .format(typ, hinter, rand))
    if typ == "QPushButton":
        stil += ("QPushButton:hover {{ background: {}; }}"
                 "QPushButton:disabled {{ background: #eeeeee; color: #999999;"
                 " border-color: #cccccc; }}".format(hover))
    return stil


def _zahl(wert: float, stellen: int) -> str:
    """Zahl mit Dezimalkomma, z. B. 1,30."""
    return "{:.{}f}".format(wert, stellen).replace(".", ",")


def spannung_zu_mm(volt: float) -> float:
    """Rechnet die Poti-Spannung linear in die Hoehe (0 mm = unten) um."""
    return (volt - POTI_U_UNTEN) / (POTI_U_OBEN - POTI_U_UNTEN) * HUB_MM


# --------------------------------------------------------------------------
# Hardwarezugriff
# --------------------------------------------------------------------------
class AOAusgang:
    """Haelt einen DAQmx-Task mit beiden Analogausgaengen eines Geraets."""

    def __init__(self):
        self.task = None
        self.geraet = ""
        self.werte = [0.0, 0.0]

    @property
    def verbunden(self) -> bool:
        return self.task is not None

    @staticmethod
    def geraete_mit_ao() -> list:
        """Namen aller DAQmx-Geraete, die Analogausgaenge besitzen."""
        if System is None:
            return []
        try:
            return [d.name for d in System.local().devices
                    if len(d.ao_physical_chans) >= 1]
        except Exception:
            return []

    def oeffnen(self, geraet: str) -> None:
        self.schliessen()
        task = nidaqmx.Task()
        try:
            for kanal in (0, 1):
                task.ao_channels.add_ao_voltage_chan(
                    geraet + "/ao" + str(kanal), min_val=AO_MIN, max_val=AO_MAX)
            task.write([0.0, 0.0], auto_start=True)
        except Exception:
            task.close()
            raise
        self.task = task
        self.geraet = geraet
        self.werte = [0.0, 0.0]

    def schreiben(self, werte) -> None:
        """Schreibt beide Ausgaenge - mit letzter Pruefung vor der Hardware."""
        if self.task is None:
            return
        werte = [float(werte[0]), float(werte[1])]
        for w in werte:
            if not AO_MIN <= w <= AO_MAX:
                self._null_ausgeben()
                raise ValueError(
                    "Sollwert {:+.3f} V ausserhalb {:g} ... {:g} V - "
                    "Ausgaenge auf 0 V gesetzt.".format(w, AO_MIN, AO_MAX))
        if werte[KANAL_HOCH] > 0.0 and werte[KANAL_RUNTER] > 0.0:
            self._null_ausgeben()
            raise RuntimeError(
                "Verriegelung: HOCH und RUNTER gleichzeitig angefordert - "
                "Ausgaenge auf 0 V gesetzt.")
        self.task.write(werte, auto_start=True)
        self.werte = werte

    def _null_ausgeben(self) -> None:
        try:
            self.task.write([0.0, 0.0], auto_start=True)
        except Exception:
            pass
        self.werte = [0.0, 0.0]

    def schliessen(self) -> None:
        """Ausgaenge auf 0 V fahren und Task freigeben."""
        if self.task is not None:
            self._null_ausgeben()
            try:
                self.task.close()
            except Exception:
                pass
            self.task = None
        self.werte = [0.0, 0.0]


class AIEingang:
    """Haelt einen DAQmx-Task fuer einen einzelnen Analogeingang."""

    def __init__(self):
        self.task = None
        self.samples = 1

    @property
    def laeuft(self) -> bool:
        return self.task is not None

    def oeffnen(self, geraet, samples) -> None:
        self.schliessen()
        task = nidaqmx.Task()
        try:
            task.ai_channels.add_ai_voltage_chan(
                geraet + "/" + AI_KANAL,
                terminal_config=TerminalConfiguration.DIFF,
                min_val=-AI_BEREICH, max_val=AI_BEREICH)
            if samples > 1:
                # Gepufferte Burst-Messung: schnell abtasten und mitteln.
                task.timing.cfg_samp_clk_timing(
                    AI_ABTASTRATE, sample_mode=AcquisitionType.FINITE,
                    samps_per_chan=samples)
        except Exception:
            task.close()
            raise
        self.task = task
        self.samples = samples

    def messen(self):
        """Liefert die Rohwerte einer Messung (Liste von Volt-Werten)."""
        if self.task is None:
            return []
        if self.samples > 1:
            werte = self.task.read(
                number_of_samples_per_channel=self.samples, timeout=5.0)
            self.task.stop()            # Task fuer den naechsten Burst scharf machen
            return list(werte)
        return [float(self.task.read())]

    def schliessen(self) -> None:
        if self.task is not None:
            try:
                self.task.close()
            except Exception:
                pass
            self.task = None


# --------------------------------------------------------------------------
# Bedienfeld fuer einen Fahrkanal
# --------------------------------------------------------------------------
class KanalWidget(QGroupBox):

    geaendert = pyqtSignal()

    def __init__(self, index: int, richtung: str):
        super().__init__("ao{}  -  {}".format(index, richtung.upper()))

        self.chk_frei = QCheckBox(richtung + " fahren")
        schrift_chk = QFont()
        schrift_chk.setBold(True)
        self.chk_frei.setFont(schrift_chk)
        self.chk_frei.toggled.connect(self.geaendert)

        self.spin_soll = QDoubleSpinBox()
        self.spin_soll.setRange(AO_MIN, AO_MAX)
        self.spin_soll.setDecimals(3)
        self.spin_soll.setSingleStep(0.1)
        self.spin_soll.setSuffix(" V")
        self.spin_soll.valueChanged.connect(self.geaendert)

        self.lbl_info = QLabel(
            "Relais schaltet ab ca. {:.1f} V".format(
                RELAIS_SCHWELLE).replace(".", ","))
        self.lbl_info.setStyleSheet("color: #666666;")

        self.lbl_hinweis = QLabel("")
        self.lbl_hinweis.setStyleSheet("color: #b9770e; font-style: italic;")

        self.lbl_ist = QLabel("—")
        schrift = QFont("Consolas", 16)
        schrift.setBold(True)
        self.lbl_ist.setFont(schrift)
        self.lbl_ist.setAlignment(Qt.AlignmentFlag.AlignCenter)

        raster = QGridLayout(self)
        raster.addWidget(QLabel("Sollwert"), 0, 0)
        raster.addWidget(self.spin_soll, 0, 1)
        raster.addWidget(self.chk_frei, 0, 2)
        raster.addWidget(self.lbl_info, 1, 0, 1, 3)
        raster.addWidget(self.lbl_hinweis, 2, 0, 1, 3)
        raster.addWidget(self.lbl_ist, 3, 0, 1, 3)

        self.spin_soll.setValue(SOLL_STANDARD)

    # -- Zustand -----------------------------------------------------------
    @property
    def freigegeben(self) -> bool:
        return self.chk_frei.isChecked()

    @property
    def sollwert(self) -> float:
        return self.spin_soll.value()

    @property
    def ausgabewert(self) -> float:
        """Was tatsaechlich ausgegeben wird: 0 V, solange nicht freigeschaltet."""
        return self.sollwert if self.freigegeben else 0.0

    def sperren(self) -> None:
        self.chk_frei.setChecked(False)

    def freigabe_erlauben(self, erlaubt: bool, grund: str = "") -> None:
        """Sperrt die Checkbox (z. B. Verriegelung). Abschalten geht immer."""
        self.chk_frei.setEnabled(erlaubt or self.freigegeben)
        self.lbl_hinweis.setText("" if erlaubt else "gesperrt: " + grund)

    def anzeige_setzen(self, volt) -> None:
        if volt is None:
            self.lbl_ist.setText("-- getrennt --")
            farbe = "#888888"
        elif self.freigegeben:
            relais = "Relais EIN" if volt >= RELAIS_SCHWELLE else "Relais AUS?"
            self.lbl_ist.setText("{:+.3f} V  {}".format(volt, relais))
            farbe = "#c0392b"
        else:
            self.lbl_ist.setText("{:+.3f} V  gestoppt".format(volt))
            farbe = "#2d7d46"
        self.lbl_ist.setStyleSheet("color: " + farbe + ";")


# --------------------------------------------------------------------------
# Positionsmessung ueber ai0
# --------------------------------------------------------------------------
class AIWidget(QGroupBox):

    meldung = pyqtSignal(str)
    zustand = pyqtSignal()          # neue Position, Start/Stopp, Grenzen geaendert
    messwert = pyqtSignal(float, float)     # z [mm], U ai0 [V]

    def __init__(self, geraet_abfrage):
        super().__init__("Position z  (Poti an " + AI_KANAL + ")")
        self.geraet_abfrage = geraet_abfrage     # Callable -> Geraetename
        self.hw = AIEingang()
        self.verlauf_fein = deque()     # (Zeit, mm) aller Messungen, max. 1 h
        self.verlauf_grob = deque()     # (Zeit, mm) alle 10 s, max. 7 Tage
        self._position = None
        self._messbeginn = None         # Zeitpunkt von "Messung starten"

        # --- Einstellungen ---
        self.spin_mittel = QSpinBox()
        self.spin_mittel.setRange(1, 10000)
        self.spin_mittel.setValue(100)
        self.spin_mittel.setSuffix(" Samples")

        self.spin_intervall = QSpinBox()
        self.spin_intervall.setRange(20, 10000)
        self.spin_intervall.setSingleStep(10)
        self.spin_intervall.setValue(100)
        self.spin_intervall.setSuffix(" ms")
        self.spin_intervall.valueChanged.connect(self._intervall_geaendert)

        self.spin_unten = QDoubleSpinBox()
        self.spin_unten.setRange(0.0, HUB_MM)
        self.spin_unten.setDecimals(1)
        self.spin_unten.setSingleStep(0.5)
        self.spin_unten.setValue(GRENZE_UNTEN_STANDARD)
        self.spin_unten.setSuffix(" mm")

        self.spin_oben = QDoubleSpinBox()
        self.spin_oben.setRange(0.0, HUB_MM)
        self.spin_oben.setDecimals(1)
        self.spin_oben.setSingleStep(0.5)
        self.spin_oben.setValue(GRENZE_OBEN_STANDARD)
        self.spin_oben.setSuffix(" mm")

        self.spin_unten.valueChanged.connect(self._grenzen_geaendert)
        self.spin_oben.valueChanged.connect(self._grenzen_geaendert)

        self.btn_start = QPushButton("Messung starten")
        self.btn_start.setStyleSheet(zustand_stil(False, "QPushButton"))
        self.btn_start.clicked.connect(self._umschalten)

        self.cmb_fenster = QComboBox()
        for text, sekunden in ZEITFENSTER:
            self.cmb_fenster.addItem(text, sekunden)
        self.cmb_fenster.setCurrentIndex(1)             # Default: 5 min
        self.cmb_fenster.currentIndexChanged.connect(self.ansicht_zuruecksetzen)

        self.btn_reset = QPushButton("Verlauf loeschen")
        self.btn_reset.clicked.connect(self._verlauf_loeschen)

        # --- Anzeige ---
        self.lbl_wert = QLabel("z = ---,-- mm")
        schrift = QFont("Consolas", 30)
        schrift.setBold(True)
        self.lbl_wert.setFont(schrift)
        self.lbl_wert.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl_wert.setStyleSheet("color: #888888;")

        self.lbl_spannung = QLabel(AI_KANAL + ":  --- V")
        schrift_u = QFont("Consolas", 12)
        schrift_u.setBold(True)
        self.lbl_spannung.setFont(schrift_u)
        self.lbl_spannung.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.lbl_stat = QLabel("min --- | max --- | s --- | n ---")
        self.lbl_stat.setFont(QFont("Consolas", 9))
        self.lbl_stat.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.lbl_kalib = QLabel(
            "Kalibrierung: {:+.2f} V = 0 mm (unten), {:+.2f} V = {:g} mm (oben)"
            .format(POTI_U_UNTEN, POTI_U_OBEN, HUB_MM))
        self.lbl_kalib.setStyleSheet("color: #666666;")
        self.lbl_kalib.setAlignment(Qt.AlignmentFlag.AlignCenter)

        if pg is not None:
            pg.setConfigOptions(antialias=True)
            self.plot = pg.PlotWidget(
                background="#ffffff",
                axisItems={"bottom": pg.DateAxisItem(orientation="bottom")})
            self.plot.showGrid(x=True, y=True, alpha=0.3)
            self.plot.setLabel("left", "z [mm]")
            self.plot.setLabel("bottom", "Uhrzeit")
            self.kurve = self.plot.plot(pen=pg.mkPen("#1f77b4", width=2))
            self.kurve.setDownsampling(auto=True, method="peak")
            self.kurve.setClipToView(True)
            stift = pg.mkPen("#c0392b", width=1, style=Qt.PenStyle.DashLine)
            self.linie_unten = pg.InfiniteLine(angle=0, pen=stift)
            self.linie_oben = pg.InfiniteLine(angle=0, pen=stift)
            self.plot.addItem(self.linie_unten)
            self.plot.addItem(self.linie_oben)
            # Zeitachse folgt dem Zeitfenster; mit der Maus wird nur z gezoomt.
            # Weiter als der ganze Hub laesst sich nicht herauszoomen.
            self.plot.setMouseEnabled(x=False, y=True)
            self.plot.setLimits(yMin=Z_ANSICHT[0] - 3.0, yMax=Z_ANSICHT[1] + 3.0,
                                minYRange=0.05)
            self.plot.hideButtons()
            self.plot.setToolTip("Mausrad: z-Achse zoomen · Ziehen: verschieben\n"
                                 "Doppelklick: Ansicht zurücksetzen")
            self.plot.scene().sigMouseClicked.connect(self._plot_geklickt)
            self.plot.setYRange(*Z_ANSICHT, padding=0)
            self.plot.setMinimumHeight(160)
        else:
            self.plot = QLabel("(pyqtgraph nicht installiert - keine Kurve)")
            self.plot.setAlignment(Qt.AlignmentFlag.AlignCenter)
            self.kurve = None
            self.linie_unten = self.linie_oben = None
        self._grenzlinien_setzen()

        bedienung = QGridLayout()
        bedienung.addWidget(QLabel("Mittelung"), 0, 0)
        bedienung.addWidget(self.spin_mittel, 0, 1)
        bedienung.addWidget(QLabel("Intervall"), 0, 2)
        bedienung.addWidget(self.spin_intervall, 0, 3)
        tipp_unten = ("RUNTER wird automatisch abgeschaltet, sobald die\n"
                      "gemessene Position diesen Wert erreicht.")
        tipp_oben = ("HOCH wird automatisch abgeschaltet, sobald die\n"
                     "gemessene Position diesen Wert erreicht\n"
                     "(Schutz vor dem oberen Anschlag).")
        self.spin_unten.setToolTip(tipp_unten)
        self.spin_oben.setToolTip(tipp_oben)
        lbl_unten = QLabel("Stopp unten bei")
        lbl_oben = QLabel("Stopp oben bei")
        lbl_unten.setToolTip(tipp_unten)
        lbl_oben.setToolTip(tipp_oben)
        bedienung.addWidget(lbl_unten, 1, 0)
        bedienung.addWidget(self.spin_unten, 1, 1)
        bedienung.addWidget(lbl_oben, 1, 2)
        bedienung.addWidget(self.spin_oben, 1, 3)
        bedienung.addWidget(self.btn_start, 0, 4, 1, 2)
        bedienung.addWidget(QLabel("Zeitfenster"), 1, 4)
        bedienung.addWidget(self.cmb_fenster, 1, 5)
        bedienung.addWidget(self.btn_reset, 2, 4, 1, 2)

        layout = QVBoxLayout(self)
        layout.addLayout(bedienung)
        layout.addWidget(self.lbl_wert)
        layout.addWidget(self.lbl_spannung)
        layout.addWidget(self.lbl_stat)
        layout.addWidget(self.lbl_kalib)
        layout.addWidget(self.plot)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._messen)

        if nidaqmx is None:
            self.btn_start.setEnabled(False)

    # -- Zustand -----------------------------------------------------------
    @property
    def laeuft(self) -> bool:
        return self.hw.laeuft

    @property
    def position_mm(self):
        """Letzte gemessene Position in mm, None wenn keine Messung laeuft."""
        return self._position if self.laeuft else None

    @property
    def grenze_unten(self) -> float:
        return self.spin_unten.value()

    @property
    def grenze_oben(self) -> float:
        return self.spin_oben.value()

    # -- Steuerung ---------------------------------------------------------
    def _umschalten(self) -> None:
        if self.laeuft:
            self.stoppen()
        else:
            self.starten()

    def starten(self) -> None:
        geraet = self.geraet_abfrage()
        samples = self.spin_mittel.value()
        try:
            self.hw.oeffnen(geraet, samples)
        except Exception as fehler:
            QMessageBox.critical(self, "Messung nicht moeglich", str(fehler))
            self.meldung.emit("FEHLER Positionsmessung: " + str(fehler))
            return
        self._position = None
        self._messbeginn = time.time()
        self.spin_mittel.setEnabled(False)
        self.btn_start.setText("Messung läuft – stoppen")
        self.btn_start.setStyleSheet(zustand_stil(True, "QPushButton"))
        self.lbl_wert.setStyleSheet("color: #1f77b4;")
        self.meldung.emit("Positionsmessung gestartet")
        self.timer.start(self.spin_intervall.value())
        self._messen()

    def stoppen(self) -> None:
        self.timer.stop()
        self.hw.schliessen()
        self._position = None
        self.spin_mittel.setEnabled(True)
        self.btn_start.setText("Messung starten")
        self.btn_start.setStyleSheet(zustand_stil(False, "QPushButton"))
        self.lbl_wert.setStyleSheet("color: #888888;")
        self.meldung.emit("Positionsmessung gestoppt")
        self.zustand.emit()

    def _intervall_geaendert(self, wert: int) -> None:
        if self.timer.isActive():
            self.timer.start(wert)

    def _grenzen_geaendert(self) -> None:
        # untere Grenze darf nicht ueber der oberen liegen
        self.spin_unten.setMaximum(self.spin_oben.value())
        self.spin_oben.setMinimum(self.spin_unten.value())
        self._grenzlinien_setzen()
        self.zustand.emit()

    def _grenzlinien_setzen(self) -> None:
        if self.linie_unten is not None:
            self.linie_unten.setValue(self.grenze_unten)
            self.linie_oben.setValue(self.grenze_oben)

    def ansicht_zuruecksetzen(self) -> None:
        """z-Achse wieder auf den ganzen Hub, Zeitachse auf das Zeitfenster."""
        if self.kurve is not None:
            self.plot.setYRange(*Z_ANSICHT, padding=0)
        self._kurve_zeichnen()

    def _plot_geklickt(self, ereignis) -> None:
        if ereignis.double():
            self.ansicht_zuruecksetzen()

    def _verlauf_loeschen(self) -> None:
        self.verlauf_fein.clear()
        self.verlauf_grob.clear()
        self._kurve_zeichnen()

    # -- Messung -----------------------------------------------------------
    def _messen(self) -> None:
        try:
            rohwerte = self.hw.messen()
        except Exception as fehler:
            self.meldung.emit("FEHLER Positionsmessung: " + str(fehler))
            self.stoppen()
            QMessageBox.critical(self, "Lesefehler", str(fehler))
            return
        if not rohwerte:
            return

        anzahl = len(rohwerte)
        mittel = sum(rohwerte) / anzahl
        kleinster = min(rohwerte)
        groesster = max(rohwerte)
        if anzahl > 1:
            varianz = sum((w - mittel) ** 2 for w in rohwerte) / (anzahl - 1)
            streuung = varianz ** 0.5
        else:
            streuung = 0.0

        mm = spannung_zu_mm(mittel)
        self._position = mm

        self.lbl_wert.setText("z = {:6.2f} mm".format(mm))
        self.lbl_spannung.setText("{}:  {:+.4f} V".format(AI_KANAL, mittel))
        self.lbl_stat.setText(
            "min {:+.4f} V | max {:+.4f} V | s {:.5f} V | n {}"
            .format(kleinster, groesster, streuung, anzahl))

        self._verlauf_speichern(mm)
        self._kurve_zeichnen()
        self.zustand.emit()
        self.messwert.emit(mm, mittel)

    def _verlauf_speichern(self, mm: float) -> None:
        jetzt = time.time()
        self.verlauf_fein.append((jetzt, mm))
        while self.verlauf_fein and self.verlauf_fein[0][0] < jetzt - VERLAUF_FEIN_S:
            self.verlauf_fein.popleft()
        if (not self.verlauf_grob
                or jetzt - self.verlauf_grob[-1][0] >= VERLAUF_GROB_RASTER):
            self.verlauf_grob.append((jetzt, mm))
            while self.verlauf_grob and self.verlauf_grob[0][0] < jetzt - VERLAUF_GROB_S:
                self.verlauf_grob.popleft()

    def _kurve_zeichnen(self) -> None:
        if self.kurve is None:
            return
        fenster = self.cmb_fenster.currentData()
        jetzt = time.time()
        if fenster == SEIT_MESSBEGINN:
            beginn = self._messbeginn
            if beginn is None:      # noch nie gemessen
                beginn = self.verlauf_grob[0][0] if self.verlauf_grob else jetzt
            start = min(beginn, jetzt - 60)     # mindestens 1 min breit
            fenster = jetzt - start
        else:
            start = jetzt - fenster
        if fenster <= VERLAUF_FEIN_S:
            punkte = [p for p in self.verlauf_fein if p[0] >= start]
        else:
            # aelterer Teil aus dem groben, letzte Stunde aus dem feinen Verlauf
            beginn_fein = (self.verlauf_fein[0][0] if self.verlauf_fein
                           else jetzt)
            punkte = [p for p in self.verlauf_grob
                      if start <= p[0] < beginn_fein]
            punkte += list(self.verlauf_fein)
        self.kurve.setData([p[0] for p in punkte], [p[1] for p in punkte])
        self.plot.setXRange(start, jetzt, padding=0)

# --------------------------------------------------------------------------
# Datenspeicherung (optional, standardmaessig aus)
# --------------------------------------------------------------------------
class Aufzeichnung(QGroupBox):
    """Schreibt Messwerte und Aktionen nur auf Wunsch in eine CSV-Datei.

    Format fuer deutsches Excel: Trennzeichen ';', Dezimalkomma.
    """

    meldung = pyqtSignal(str)

    def __init__(self):
        super().__init__("Datenspeicherung")
        self.datei = None
        self.pfad = ""
        self.zeilen = 0
        self._letzte_zeit = None
        self._fuhr = False
        self.ordner = os.path.join(os.path.expanduser("~"), "Documents")

        # --- Optionen ---
        # Was gespeichert wird: (Positionsdaten, Steuerbefehle)
        self.cmb_inhalt = QComboBox()
        self.cmb_inhalt.addItem("Positionsdaten + Steuerbefehle", (True, True))
        self.cmb_inhalt.addItem("Nur Positionsdaten", (True, False))
        self.cmb_inhalt.addItem("Nur Steuerbefehle", (False, True))
        self.cmb_inhalt.setToolTip(
            "Positionsdaten: z [mm], Spannung ai0 und die Spannungen an ao0/ao1.\n"
            "Steuerbefehle: z. B. „HOCH fahren mit 1,30 V“, Stopps, NOT-AUS.")
        self.cmb_inhalt.currentIndexChanged.connect(self._inhalt_geaendert)

        self.cmb_intervall = QComboBox()
        for text, sekunden in SPEICHER_INTERVALLE:
            self.cmb_intervall.addItem(text, sekunden)
        self.cmb_intervall.setCurrentIndex(SPEICHER_INTERVALL_STANDARD)

        self.cmb_wann = QComboBox()
        self.cmb_wann.addItem("immer", False)
        self.cmb_wann.addItem("nur während der Motor fährt", True)
        self.cmb_wann.setToolTip(
            "„nur während der Motor fährt“: Start- und Endposition\n"
            "jeder Fahrt werden dabei immer gespeichert.")

        self.lbl_position = QLabel("Position speichern:")

        # --- Start/Stopp (Farbe zeigt den Zustand) ---
        self.btn = QPushButton()
        self.btn.setMinimumHeight(30)
        self.btn.clicked.connect(self._umschalten)

        self.lbl = QLabel()
        self.lbl.setStyleSheet("color: #666666;")
        self.lbl.setTextInteractionFlags(
            Qt.TextInteractionFlag.TextSelectableByMouse)

        optionen = QHBoxLayout()
        optionen.addWidget(QLabel("Was speichern:"))
        optionen.addWidget(self.cmb_inhalt)
        optionen.addSpacing(24)
        optionen.addWidget(self.lbl_position)
        optionen.addWidget(self.cmb_intervall)
        optionen.addWidget(self.cmb_wann)
        optionen.addStretch(1)

        kopf = QHBoxLayout()
        kopf.addWidget(self.btn)
        kopf.addWidget(self.lbl, 1)

        layout = QVBoxLayout(self)
        layout.addLayout(kopf)
        layout.addLayout(optionen)

        self._optik(False)

    @property
    def laeuft(self) -> bool:
        return self.datei is not None

    def _umschalten(self) -> None:
        if self.laeuft:
            self.stoppen()
        else:
            self.starten()

    def starten(self) -> None:
        name = "z_position" if self._position_speichern else "z_steuerbefehle"
        vorschlag = os.path.join(
            self.ordner, time.strftime(name + "_%Y-%m-%d_%H-%M-%S.csv"))
        pfad, _ = QFileDialog.getSaveFileName(
            self, "Aufzeichnung speichern unter", vorschlag,
            "CSV-Dateien (*.csv)")
        if not pfad:
            return
        try:
            self.datei = open(pfad, "w", encoding="utf-8-sig", newline="")
            self.datei.write(
                ";".join(["Datum", "Uhrzeit"] + self._spalten()) + "\n")
        except OSError as fehler:
            self.datei = None
            QMessageBox.critical(self, "Speichern nicht möglich", str(fehler))
            self.meldung.emit("FEHLER Speichern: " + str(fehler))
            return
        self.pfad = pfad
        self.ordner = os.path.dirname(pfad)
        self.zeilen = 0
        self._letzte_zeit = None
        self._fuhr = False
        self._optik(True)
        self._anzeige()
        self.meldung.emit("Aufzeichnung gestartet: " + os.path.basename(pfad))

    def stoppen(self) -> None:
        if not self.laeuft:
            return
        try:
            self.datei.close()
        except OSError:
            pass
        self.datei = None
        self._optik(False)
        self.lbl.setText("Zuletzt gespeichert: {}  ·  {}".format(
            os.path.basename(self.pfad), self._zeilen_text()))
        self.meldung.emit("Aufzeichnung gestoppt: {} ({})".format(
            os.path.basename(self.pfad), self._zeilen_text()))

    def messwert(self, mm: float, volt: float, ao0: float, ao1: float) -> None:
        """Wird bei jeder Positionsmessung aufgerufen."""
        if not self.laeuft or not self._position_speichern:
            return
        faehrt = ao0 > 0.0 or ao1 > 0.0
        gerade_gestoppt = self._fuhr and not faehrt
        gerade_losgefahren = faehrt and not self._fuhr
        self._fuhr = faehrt
        if self.cmb_wann.currentData() and not faehrt and not gerade_gestoppt:
            return
        jetzt = time.time()
        intervall = self.cmb_intervall.currentData()
        # Start und Ende einer Fahrt immer speichern, sonst im gewaehlten Takt
        if (intervall and self._letzte_zeit is not None
                and jetzt - self._letzte_zeit < intervall
                and not gerade_gestoppt and not gerade_losgefahren):
            return
        self._letzte_zeit = jetzt
        felder = [_zahl(mm, 3), _zahl(volt, 5), _zahl(ao0, 3), _zahl(ao1, 3)]
        if self.cmb_inhalt.currentData()[1]:
            felder.append("")                   # leere Spalte Steuerbefehl
        self._schreiben(jetzt, felder)

    def aktion(self, text: str) -> None:
        if self.laeuft and self.cmb_inhalt.currentData()[1]:
            leer = ["", "", "", ""] if self._position_speichern else []
            self._schreiben(time.time(), leer + [text])

    def _spalten(self) -> list:
        """Spaltenkoepfe passend zur Auswahl 'Was speichern'."""
        position, befehle = self.cmb_inhalt.currentData()
        spalten = []
        if position:
            spalten += ["z [mm]", "U ai0 [V]", "ao0 hoch [V]", "ao1 runter [V]"]
        if befehle:
            spalten.append("Steuerbefehl")
        return spalten

    @property
    def _position_speichern(self) -> bool:
        return self.cmb_inhalt.currentData()[0]

    def _inhalt_geaendert(self) -> None:
        # Positions-Optionen nur, wenn auch Positionsdaten gespeichert werden
        an = self._position_speichern and not self.laeuft
        for w in (self.lbl_position, self.cmb_intervall, self.cmb_wann):
            w.setEnabled(an)

    def _schreiben(self, zeit: float, felder) -> None:
        t = time.localtime(zeit)
        zeile = [time.strftime("%Y-%m-%d", t),
                 time.strftime("%H:%M:%S", t)
                 + ",{:03d}".format(int((zeit % 1) * 1000))] + felder
        try:
            self.datei.write(";".join(zeile) + "\n")
            self.datei.flush()
        except OSError as fehler:
            self.meldung.emit("FEHLER Speichern: " + str(fehler))
            self.stoppen()
            return
        self.zeilen += 1
        self._anzeige()

    def _optik(self, aktiv: bool) -> None:
        """Knopf und Optionen je nach Zustand gestalten."""
        self.cmb_inhalt.setEnabled(not aktiv)   # Optionen nur vor dem Start
        self._inhalt_geaendert()
        if aktiv:
            self.btn.setText("Aufzeichnung läuft – beenden")
        else:
            self.btn.setText("Aufzeichnung starten …")
            self.lbl.setText("Es werden keine Daten gespeichert.")
        self.btn.setStyleSheet(zustand_stil(aktiv, "QPushButton"))

    def _zeilen_text(self) -> str:
        return "1 Zeile" if self.zeilen == 1 else "{} Zeilen".format(self.zeilen)

    def _anzeige(self) -> None:
        self.lbl.setText("{}  ·  {}".format(
            os.path.basename(self.pfad), self._zeilen_text()))
        self.lbl.setToolTip(self.pfad)


# --------------------------------------------------------------------------
# Position anfahren / Positionen merken
# --------------------------------------------------------------------------
class PositionAnfahren(QGroupBox):
    """Zielposition eingeben oder gemerkte Position waehlen und anfahren.

    Gemerkte Positionen und der gelernte Nachlauf werden in
    POSITIONEN_DATEI gespeichert und beim naechsten Start wieder geladen.
    """

    anfahren = pyqtSignal(float, str)   # Ziel [mm], Name (oder "")
    abbrechen = pyqtSignal()
    meldung = pyqtSignal(str)

    def __init__(self, position_abfrage):
        super().__init__("Position anfahren")
        self.position_abfrage = position_abfrage    # Callable -> mm oder None
        self.positionen = []                # [(Name, mm)], nach mm sortiert
        self.vorhalt = [0.0, 0.0]           # gelernter Nachlauf HOCH / RUNTER
        self._laeuft = False
        self._laden()

        self.spin_ziel = QDoubleSpinBox()
        self.spin_ziel.setRange(0.0, HUB_MM)
        self.spin_ziel.setDecimals(2)
        self.spin_ziel.setSingleStep(0.1)
        self.spin_ziel.setSuffix(" mm")
        self.spin_ziel.setValue(HUB_MM / 2)

        self.btn_fahren = QPushButton("Anfahren")
        self.btn_fahren.setMinimumWidth(140)
        self.btn_fahren.clicked.connect(self._fahren_geklickt)

        self.lbl_status = QLabel("")

        self.cmb_gemerkt = QComboBox()
        self.cmb_gemerkt.setPlaceholderText("– gemerkte Position wählen –")
        self.cmb_gemerkt.setMinimumWidth(220)
        self.cmb_gemerkt.currentIndexChanged.connect(self._gemerkt_gewaehlt)

        # faehrt die in der Liste gewaehlte Position direkt an
        self.btn_gemerkt_fahren = QPushButton("Anfahren")
        self.btn_gemerkt_fahren.setMinimumWidth(140)
        self.btn_gemerkt_fahren.clicked.connect(self._gemerkt_fahren)

        self._erlaubt = False
        self._grund = ""

        self.btn_merken = QPushButton("Aktuelle Position merken …")
        self.btn_merken.clicked.connect(self._merken)
        self.btn_loeschen = QPushButton("Löschen")
        self.btn_loeschen.clicked.connect(self._loeschen)

        self.lbl_nachlauf = QLabel()
        self.lbl_nachlauf.setStyleSheet("color: #666666;")
        self.lbl_nachlauf.setToolTip(
            "Der Motor läuft nach dem Abschalten noch ein Stück weiter.\n"
            "Dieser Nachlauf wird bei jeder Anfahrt gemessen und gelernt;\n"
            "abgeschaltet wird entsprechend früher.")

        raster = QGridLayout(self)
        raster.addWidget(QLabel("Ziel:"), 0, 0)
        raster.addWidget(self.spin_ziel, 0, 1)
        raster.addWidget(self.btn_fahren, 0, 2)
        raster.addWidget(self.lbl_status, 0, 3, 1, 3)
        raster.addWidget(QLabel("Gemerkt:"), 1, 0)
        raster.addWidget(self.cmb_gemerkt, 1, 1)
        raster.addWidget(self.btn_gemerkt_fahren, 1, 2)
        raster.addWidget(self.btn_merken, 1, 3)
        raster.addWidget(self.btn_loeschen, 1, 4)
        raster.addWidget(self.lbl_nachlauf, 1, 5)
        raster.setColumnStretch(5, 1)

        self._liste_fuellen()
        self._nachlauf_anzeigen()
        self.zustand_setzen(False, False, "nicht verbunden")

    # -- Zustand von aussen ------------------------------------------------
    def zustand_setzen(self, laeuft: bool, erlaubt: bool, grund: str = "") -> None:
        """laeuft: Anfahrt aktiv. erlaubt: eine neue Anfahrt ist moeglich."""
        self._laeuft = laeuft
        self._erlaubt = erlaubt
        self._grund = grund
        self._knoepfe_setzen()
        self.spin_ziel.setEnabled(not laeuft)
        self.cmb_gemerkt.setEnabled(not laeuft and bool(self.positionen))
        self.btn_merken.setEnabled(self.position_abfrage() is not None)
        self.btn_merken.setToolTip(
            "" if self.btn_merken.isEnabled() else "erst Positionsmessung starten")
        self.btn_loeschen.setEnabled(
            not laeuft and self.cmb_gemerkt.currentIndex() >= 0)

    def status(self, text: str, farbe: str = "#333333") -> None:
        self.lbl_status.setText(text)
        self.lbl_status.setStyleSheet("color: " + farbe + ";")

    def vorhalt_lernen(self, index: int, nachlauf: float) -> None:
        """Gemessenen Nachlauf (mm, in Fahrtrichtung) einrechnen."""
        neu = 0.5 * self.vorhalt[index] + 0.5 * nachlauf
        self.vorhalt[index] = min(max(neu, 0.0), VORHALT_MAX)
        self._nachlauf_anzeigen()
        self._speichern()

    # -- Bedienung ---------------------------------------------------------
    def _fahren_geklickt(self) -> None:
        if self._laeuft:
            self.abbrechen.emit()
            return
        ziel = round(self.spin_ziel.value(), 2)
        name = ""
        i = self.cmb_gemerkt.currentIndex()
        if i >= 0 and abs(self.positionen[i][1] - ziel) < 0.005:
            name = self.positionen[i][0]
        self.anfahren.emit(ziel, name)

    def _gemerkt_fahren(self) -> None:
        if self._laeuft:
            self.abbrechen.emit()
            return
        i = self.cmb_gemerkt.currentIndex()
        if 0 <= i < len(self.positionen):
            name, mm = self.positionen[i]
            self.spin_ziel.setValue(mm)
            self.anfahren.emit(mm, name)

    def _gemerkt_gewaehlt(self, i: int) -> None:
        if 0 <= i < len(self.positionen):
            self.spin_ziel.setValue(self.positionen[i][1])
        self.btn_loeschen.setEnabled(not self._laeuft and i >= 0)
        self._knoepfe_setzen()

    def _knoepfe_setzen(self) -> None:
        """Beide Anfahren-Knoepfe; waehrend einer Anfahrt = abbrechen."""
        gewaehlt = self.cmb_gemerkt.currentIndex() >= 0
        for btn, moeglich, grund in (
                (self.btn_fahren, self._erlaubt, self._grund),
                (self.btn_gemerkt_fahren, self._erlaubt and gewaehlt,
                 self._grund if not self._erlaubt
                 else "erst eine gemerkte Position wählen")):
            if self._laeuft:
                btn.setText("Anfahrt abbrechen")
                btn.setEnabled(True)
                btn.setToolTip("")
            else:
                btn.setText("Anfahren")
                btn.setEnabled(moeglich)
                btn.setToolTip("" if moeglich else "gesperrt: " + grund)

    def _merken(self) -> None:
        pos = self.position_abfrage()
        if pos is None:
            return
        pos = round(pos, 2)
        vorschlag = "Position " + str(len(self.positionen) + 1)
        name, ok = QInputDialog.getText(
            self, "Position merken",
            "Name für z = " + _zahl(pos, 2) + " mm:", text=vorschlag)
        name = name.strip()
        if not ok or not name:
            return
        ersetzt = any(n == name for n, _ in self.positionen)
        self.positionen = [(n, mm) for n, mm in self.positionen if n != name]
        self.positionen.append((name, pos))
        self.positionen.sort(key=lambda p: p[1])
        self._speichern()
        self._liste_fuellen(name)
        self.meldung.emit("Position {} „{}“: {} mm".format(
            "aktualisiert" if ersetzt else "gemerkt", name, _zahl(pos, 2)))

    def _loeschen(self) -> None:
        i = self.cmb_gemerkt.currentIndex()
        if not 0 <= i < len(self.positionen):
            return
        name, mm = self.positionen.pop(i)
        self._speichern()
        self._liste_fuellen()
        self.meldung.emit("Gemerkte Position gelöscht: „{}“ ({} mm)".format(
            name, _zahl(mm, 2)))

    # -- intern ------------------------------------------------------------
    def _liste_fuellen(self, auswahl: str = "") -> None:
        self.cmb_gemerkt.clear()
        for name, mm in self.positionen:
            self.cmb_gemerkt.addItem("{}  ({} mm)".format(name, _zahl(mm, 2)))
        index = next((i for i, (n, _) in enumerate(self.positionen)
                      if n == auswahl), -1)
        self.cmb_gemerkt.setCurrentIndex(index)
        self.cmb_gemerkt.setEnabled(not self._laeuft and bool(self.positionen))
        self.btn_loeschen.setEnabled(not self._laeuft and index >= 0)

    def _nachlauf_anzeigen(self) -> None:
        self.lbl_nachlauf.setText(
            "Nachlauf: hoch {} mm · runter {} mm".format(
                _zahl(self.vorhalt[KANAL_HOCH], 2),
                _zahl(self.vorhalt[KANAL_RUNTER], 2)))

    def _laden(self) -> None:
        try:
            with open(POSITIONEN_DATEI, encoding="utf-8") as datei:
                daten = json.load(datei)
            self.positionen = sorted(
                ((str(p["name"]), float(p["mm"])) for p in daten["positionen"]),
                key=lambda p: p[1])
            self.vorhalt = [min(max(float(v), 0.0), VORHALT_MAX)
                            for v in daten.get("nachlauf_mm", [0.0, 0.0])][:2]
        except FileNotFoundError:
            pass
        except (OSError, ValueError, KeyError, TypeError):
            self.positionen = []
            self.vorhalt = [0.0, 0.0]

    def _speichern(self) -> None:
        daten = {
            "positionen": [{"name": n, "mm": mm} for n, mm in self.positionen],
            "nachlauf_mm": self.vorhalt,
        }
        try:
            with open(POSITIONEN_DATEI, "w", encoding="utf-8") as datei:
                json.dump(daten, datei, ensure_ascii=False, indent=2)
        except OSError as fehler:
            self.meldung.emit("FEHLER Positionen speichern: " + str(fehler))


# --------------------------------------------------------------------------
# Hauptfenster
# --------------------------------------------------------------------------
class Hauptfenster(QWidget):

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Verfahreinheit z - PCIe-6321")
        self.hw = AOAusgang()
        self._vorher = [0.0, 0.0]           # zuletzt ausgegebene Spannungen
        self._stopp_grund = [None, None]    # Grund fuer automatischen Stopp
        self._anfahrt = None                # laufende Anfahrt (dict) oder None
        self._messung_nr = 0                # zaehlt neue Positionsmessungen

        # Kopfzeile: Geraetewahl + Verbindung
        self.cmb_geraet = QComboBox()
        self.cmb_geraet.addItems(AOAusgang.geraete_mit_ao() or ["(kein Geraet)"])
        self.btn_verbinden = QPushButton("Verbinden")
        self.btn_verbinden.clicked.connect(self._verbindung_umschalten)
        self.lbl_status = QLabel()
        self.lbl_status.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._status_farbe(False)

        kopf = QHBoxLayout()
        kopf.addWidget(QLabel("Geraet:"))
        kopf.addWidget(self.cmb_geraet)
        kopf.addWidget(self.btn_verbinden)
        kopf.addWidget(self.lbl_status)
        kopf.addStretch(1)

        # Fahrkanaele
        self.kanaele = [KanalWidget(KANAL_HOCH, "Hoch"),
                        KanalWidget(KANAL_RUNTER, "Runter")]
        self.kanal_hoch = self.kanaele[KANAL_HOCH]
        self.kanal_runter = self.kanaele[KANAL_RUNTER]
        for kanal in self.kanaele:
            kanal.geaendert.connect(self._ausgeben)
            kanal.setEnabled(False)

        kanalzeile = QHBoxLayout()
        for kanal in self.kanaele:
            kanalzeile.addWidget(kanal)

        # Endlagen-Fahrt: schaltet die Richtung ein, gestoppt wird automatisch
        # an "Stopp oben/unten bei" (bzw. unten am mechanischen Endschalter)
        self.btn_ganz_oben = QPushButton("▲  Ganz nach oben")
        self.btn_ganz_unten = QPushButton("▼  Ganz nach unten")
        endlagen = QHBoxLayout()
        for btn, kanal, ziel in ((self.btn_ganz_oben, self.kanal_hoch, "oben"),
                                 (self.btn_ganz_unten, self.kanal_runter, "unten")):
            btn.setMinimumHeight(36)
            btn.setStyleSheet("QPushButton { font-weight: bold; font-size: 13px; }")
            btn.clicked.connect(
                lambda _=False, k=kanal, z=ziel: self._endlage_fahren(k, z))
            endlagen.addWidget(btn)

        # NOT-AUS
        self.btn_notaus = QPushButton("NOT-AUS  -  Motor stopp, beide Ausgaenge auf 0 V")
        self.btn_notaus.setMinimumHeight(48)
        self.btn_notaus.setStyleSheet(
            "QPushButton { background: #c0392b; color: white; "
            "font-weight: bold; font-size: 14px; border-radius: 6px; }"
            "QPushButton:hover { background: #e74c3c; }")
        self.btn_notaus.clicked.connect(self._notaus)

        # Positionsmessung ai0 (lesend, unabhaengig vom AO-Task)
        self.ai_widget = AIWidget(self.cmb_geraet.currentText)
        self.ai_widget.meldung.connect(self._melden)
        self.ai_widget.zustand.connect(self._position_geaendert)

        # Position anfahren / merken
        self.anfahren = PositionAnfahren(lambda: self.ai_widget.position_mm)
        self.anfahren.anfahren.connect(self._anfahrt_starten)
        self.anfahren.abbrechen.connect(self._anfahrt_abbrechen)
        self.anfahren.meldung.connect(self._melden)

        # Datenspeicherung (aus, bis man sie startet)
        self.aufzeichnung = Aufzeichnung()
        self.aufzeichnung.meldung.connect(self._melden)
        self.ai_widget.messwert.connect(
            lambda mm, volt: self.aufzeichnung.messwert(
                mm, volt, self.hw.werte[KANAL_HOCH], self.hw.werte[KANAL_RUNTER]))

        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumHeight(90)
        self.log.setFont(QFont("Consolas", 9))

        layout = QVBoxLayout(self)
        layout.addLayout(kopf)
        layout.addLayout(kanalzeile)
        layout.addLayout(endlagen)
        layout.addWidget(self.anfahren)
        layout.addWidget(self.btn_notaus)
        layout.addWidget(self.ai_widget, 1)
        layout.addWidget(self.aufzeichnung)
        layout.addWidget(self.log)

        if nidaqmx is None:
            self._melden("FEHLER: Paket 'nidaqmx' ist nicht installiert")
            self.btn_verbinden.setEnabled(False)
        self._sicherheit_pruefen()
        self._anzeige_aktualisieren()

    # -- Verbindung --------------------------------------------------------
    def _verbindung_umschalten(self) -> None:
        if self.hw.verbunden:
            self._trennen()
        else:
            self._verbinden()

    def _verbinden(self) -> None:
        geraet = self.cmb_geraet.currentText()
        try:
            self.hw.oeffnen(geraet)
        except Exception as fehler:
            QMessageBox.critical(self, "Verbindung fehlgeschlagen", str(fehler))
            self._melden("FEHLER Verbinden: " + str(fehler))
            return
        self.cmb_geraet.setEnabled(False)
        self.btn_verbinden.setText("Trennen")
        self._status_farbe(True)
        for kanal in self.kanaele:
            kanal.setEnabled(True)
        self._vorher = [0.0, 0.0]
        self._melden("Verbunden mit " + geraet)
        self._sicherheit_pruefen()
        self._anzeige_aktualisieren()

    def _trennen(self) -> None:
        for kanal in self.kanaele:
            self._still_sperren(kanal)
            kanal.setEnabled(False)
        self.hw.schliessen()
        self.cmb_geraet.setEnabled(True)
        self.btn_verbinden.setText("Verbinden")
        self._status_farbe(False)
        fuhr = [n for n, u in zip(("HOCH", "RUNTER"), self._vorher) if u > 0]
        self._vorher = [0.0, 0.0]
        self._stopp_grund = [None, None]
        self._melden("Getrennt" + (" - " + fuhr[0] + " gestoppt" if fuhr else ""))
        self._sicherheit_pruefen()
        self._anzeige_aktualisieren()

    def _status_farbe(self, verbunden: bool) -> None:
        """Statusfeld: hellgruen = verbunden, hellorange = nicht verbunden."""
        text = "Verbunden" if verbunden else "Nicht verbunden"
        self.lbl_status.setText(text)
        self.lbl_status.setStyleSheet(zustand_stil(verbunden, "QLabel"))

    # -- Sicherheit --------------------------------------------------------
    @staticmethod
    def _still_sperren(kanal: KanalWidget) -> None:
        """Freigabe wegnehmen, ohne sofort eine Ausgabe auszuloesen."""
        kanal.blockSignals(True)
        kanal.sperren()
        kanal.blockSignals(False)

    def _sicherheit_pruefen(self) -> bool:
        """Verriegelung und Endlagen pruefen.

        Nimmt unzulaessige Freigaben zurueck und sperrt die Checkboxen,
        die gerade nicht benutzt werden duerfen. Liefert True, wenn eine
        Freigabe zurueckgenommen wurde (dann muss neu ausgegeben werden).
        """
        hoch, runter = self.kanal_hoch, self.kanal_runter
        pos = self.ai_widget.position_mm
        oben = self.ai_widget.grenze_oben
        unten = self.ai_widget.grenze_unten
        geaendert = self._anfahrt_pruefen()

        if hoch.freigegeben and runter.freigegeben:
            self._still_sperren(hoch)
            self._still_sperren(runter)
            self._stopp_grund = ["Verriegelung", "Verriegelung"]
            geaendert = True

        if hoch.freigegeben and pos is not None:
            if pos >= oben:
                self._still_sperren(hoch)
                self._stopp_grund[KANAL_HOCH] = (
                    "obere Grenze " + _zahl(oben, 1) + " mm erreicht")
                geaendert = True

        if runter.freigegeben and pos is not None and pos <= unten:
            self._still_sperren(runter)
            self._stopp_grund[KANAL_RUNTER] = (
                "untere Grenze " + _zahl(unten, 1) + " mm erreicht")
            geaendert = True

        # Welche Freigaben sind im Moment ueberhaupt moeglich?
        if runter.freigegeben:
            hoch.freigabe_erlauben(False, "RUNTER ist aktiv")
        elif pos is not None and pos >= oben:
            hoch.freigabe_erlauben(False, "obere Grenze erreicht")
        else:
            hoch.freigabe_erlauben(True)

        if hoch.freigegeben:
            runter.freigabe_erlauben(False, "HOCH ist aktiv")
        elif pos is not None and pos <= unten:
            runter.freigabe_erlauben(False, "untere Grenze erreicht")
        else:
            runter.freigabe_erlauben(True)

        self._endlagen_knoepfe_aktualisieren()
        self._anfahren_aktualisieren()
        return geaendert

    def _endlagen_knoepfe_aktualisieren(self) -> None:
        """Knoepfe nur bedienbar, wenn die Richtung gerade eingeschaltet werden darf."""
        oben = _zahl(self.ai_widget.grenze_oben, 1)
        unten = _zahl(self.ai_widget.grenze_unten, 1)
        for btn, kanal, tipp in (
                (self.btn_ganz_oben, self.kanal_hoch,
                 "HOCH mit dem Sollwert von ao0 einschalten.\n"
                 "Stoppt automatisch bei " + oben + " mm (Stopp oben bei)."),
                (self.btn_ganz_unten, self.kanal_runter,
                 "RUNTER mit dem Sollwert von ao1 einschalten.\n"
                 "Stoppt automatisch bei " + unten + " mm (Stopp unten bei),\n"
                 "ohne Positionsmessung am mechanischen Endschalter.")):
            erlaubt = kanal.chk_frei.isEnabled() and not kanal.freigegeben
            grund = kanal.lbl_hinweis.text()
            if kanal is self.kanal_hoch and self.ai_widget.position_mm is None:
                # ohne Messung gaebe es oben keinen automatischen Stopp
                erlaubt = False
                grund = "erst Positionsmessung starten"
            btn.setEnabled(erlaubt)
            btn.setToolTip(tipp if erlaubt or not grund else grund)

    def _endlage_fahren(self, kanal: KanalWidget, ziel: str) -> None:
        if not kanal.chk_frei.isEnabled() or kanal.freigegeben:
            return
        if kanal is self.kanal_hoch and self.ai_widget.position_mm is None:
            return
        self._melden("Fahrt ganz nach " + ziel)
        kanal.chk_frei.setChecked(True)

    # -- Position anfahren --------------------------------------------------
    def _anfahrt_starten(self, ziel: float, name: str) -> None:
        if self._anfahrt is not None:
            return
        pos = self.ai_widget.position_mm
        unten, oben = self.ai_widget.grenze_unten, self.ai_widget.grenze_oben
        if pos is None or not self.hw.verbunden:
            return
        if not unten <= ziel <= oben:
            self.anfahren.status(
                "Ziel liegt außerhalb der Stopp-Grenzen ({} … {} mm)".format(
                    _zahl(unten, 1), _zahl(oben, 1)), "#c0392b")
            return
        ziel_text = _zahl(ziel, 2) + " mm" + (" („" + name + "“)" if name else "")
        if abs(ziel - pos) <= ZIEL_TOLERANZ:
            self.anfahren.status("steht bereits auf " + _zahl(pos, 2) + " mm",
                                 "#2d7d46")
            return
        index = KANAL_HOCH if ziel > pos else KANAL_RUNTER
        kanal = self.kanaele[index]
        if not kanal.chk_frei.isEnabled() or kanal.freigegeben:
            return
        self._anfahrt = {"ziel": ziel, "text": ziel_text, "index": index,
                         "phase": "fahren", "stopp_pos": None, "stopp_zeit": None}
        self._melden("Anfahren: " + ziel_text)
        self.anfahren.status("fährt auf " + _zahl(ziel, 2) + " mm …", "#c0392b")
        kanal.chk_frei.setChecked(True)

    def _anfahrt_abbrechen(self) -> None:
        a = self._anfahrt
        if a is None or a["phase"] != "fahren":
            return
        self._anfahrt = None
        self._still_sperren(self.kanaele[a["index"]])
        self._stopp_grund[a["index"]] = "Anfahrt abgebrochen"
        self.anfahren.status("abgebrochen", "#b9770e")
        self._ausgeben()

    def _anfahrt_pruefen(self) -> bool:
        """Ziel erreicht? Dann abschalten. Liefert True, wenn gestoppt wurde."""
        a = self._anfahrt
        if a is None:
            return False
        pos = self.ai_widget.position_mm
        kanal = self.kanaele[a["index"]]
        richtung = 1.0 if a["index"] == KANAL_HOCH else -1.0

        if a["phase"] == "fahren":
            if not kanal.freigegeben:
                # von aussen gestoppt: NOT-AUS, Haken, Grenze, Trennen ...
                self._anfahrt = None
                self._melden("Anfahrt abgebrochen: " + a["text"])
                self.anfahren.status("abgebrochen", "#b9770e")
                return False
            if pos is None:
                self._anfahrt = None
                self._still_sperren(kanal)
                self._stopp_grund[a["index"]] = "Positionsmessung aus"
                self.anfahren.status("abgebrochen: Messung aus", "#b9770e")
                return True
            vorhalt = self.anfahren.vorhalt[a["index"]]
            if (pos - a["ziel"]) * richtung >= -vorhalt:
                self._still_sperren(kanal)
                self._stopp_grund[a["index"]] = "Ziel " + a["text"]
                a.update(phase="nachlauf", stopp_pos=pos,
                         stopp_zeit=time.monotonic(),
                         stopp_nr=self._messung_nr)
                self.anfahren.status("Ziel erreicht, Achse läuft aus …",
                                     "#b9770e")
                return True
            return False

        # Nachlauf: warten, bis die Achse steht, dann Ergebnis melden
        if pos is None:
            self._anfahrt = None
            self.anfahren.status("")
            return False
        # erst mit einer spaeteren Messung und nach NACHLAUF_S auswerten
        if (self._messung_nr <= a["stopp_nr"]
                or time.monotonic() - a["stopp_zeit"] < NACHLAUF_S):
            return False
        self._anfahrt = None
        abweichung = pos - a["ziel"]
        self.anfahren.vorhalt_lernen(
            a["index"], (pos - a["stopp_pos"]) * richtung)
        ergebnis = "z = {} mm (Abweichung {} mm)".format(
            _zahl(pos, 2), ("+" if abweichung >= 0 else "") + _zahl(abweichung, 2))
        self._melden("Ziel " + a["text"] + " erreicht: " + ergebnis)
        self.anfahren.status("erreicht: " + ergebnis, "#2d7d46")
        return False

    def _anfahren_aktualisieren(self) -> None:
        a = self._anfahrt
        if not self.hw.verbunden:
            erlaubt, grund = False, "nicht verbunden"
        elif self.ai_widget.position_mm is None:
            erlaubt, grund = False, "erst Positionsmessung starten"
        elif a is not None:
            erlaubt, grund = False, "Achse läuft noch aus"
        elif any(k.freigegeben for k in self.kanaele):
            erlaubt, grund = False, "Motor läuft – erst stoppen"
        else:
            erlaubt, grund = True, ""
        laeuft = a is not None and a["phase"] == "fahren"
        self.anfahren.zustand_setzen(laeuft, erlaubt, grund)

    def _position_geaendert(self) -> None:
        self._messung_nr += 1
        if self._sicherheit_pruefen():
            self._ausgeben()

    # -- Ausgabe -----------------------------------------------------------
    def _ausgeben(self) -> None:
        self._sicherheit_pruefen()
        if not self.hw.verbunden:
            self._anzeige_aktualisieren()
            return
        werte = [kanal.ausgabewert for kanal in self.kanaele]
        try:
            self.hw.schreiben(werte)
        except Exception as fehler:
            self._melden("FEHLER Ausgabe: " + str(fehler))
            QMessageBox.critical(self, "Schreibfehler", str(fehler))
            self._trennen()
            return
        self._aenderungen_melden(werte)
        self._anzeige_aktualisieren()

    def _aenderungen_melden(self, werte) -> None:
        """Meldet kurz, was sich an HOCH/RUNTER geaendert hat."""
        pos = self.ai_widget.position_mm
        ort = "" if pos is None else "  (z = " + _zahl(pos, 2) + " mm)"
        for i, name in ((KANAL_HOCH, "HOCH"), (KANAL_RUNTER, "RUNTER")):
            alt, neu = self._vorher[i], werte[i]
            if alt == neu:
                continue
            if alt == 0.0:
                text = name + " fahren mit " + _zahl(neu, 2) + " V"
            elif neu == 0.0:
                grund = self._stopp_grund[i]
                text = name + " gestoppt" + (": " + grund if grund else "")
            else:
                text = (name + " Sollwert " + _zahl(alt, 2) + " V -> "
                        + _zahl(neu, 2) + " V")
            self._melden(text + ort)
        self._vorher = list(werte)
        self._stopp_grund = [None, None]

    def _notaus(self) -> None:
        aktiv = any(k.freigegeben for k in self.kanaele)
        for kanal in self.kanaele:
            self._still_sperren(kanal)
        self._stopp_grund = ["NOT-AUS", "NOT-AUS"]
        if not aktiv:
            self._melden("NOT-AUS")
        self._ausgeben()

    def _anzeige_aktualisieren(self) -> None:
        for i, kanal in enumerate(self.kanaele):
            kanal.anzeige_setzen(self.hw.werte[i] if self.hw.verbunden else None)

    def _melden(self, text: str) -> None:
        zeile = time.strftime("%H:%M:%S") + "  " + text
        self.log.appendPlainText(zeile)
        self.aufzeichnung.aktion(text)

    # -- Programmende ------------------------------------------------------
    def closeEvent(self, event) -> None:
        if self.hw.verbunden and any(k.freigegeben for k in self.kanaele):
            antwort = QMessageBox.question(
                self, "Beenden",
                "Der Motor faehrt gerade (ein Ausgang ist freigeschaltet).\n"
                "Beim Beenden werden beide Ausgaenge auf 0 V gesetzt. Fortfahren?")
            if antwort != QMessageBox.StandardButton.Yes:
                event.ignore()
                return
        if self.ai_widget.laeuft:
            self.ai_widget.stoppen()
        self.hw.schliessen()
        self.aufzeichnung.stoppen()
        event.accept()


def main() -> int:
    app = QApplication(sys.argv)
    fenster = Hauptfenster()
    fenster.resize(860, 860)
    fenster.show()
    return app.exec()


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception:
        traceback.print_exc()
        sys.exit(1)
