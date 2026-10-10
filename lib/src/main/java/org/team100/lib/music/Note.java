package org.team100.lib.music;

import java.util.Map;
import java.util.TreeMap;

/**
 * MIDI notes from C3 to C5, equal temperament.
 * 
 * https://audiodev.blog/midi-note-chart/
 */
public enum Note {
    C3(48, 131),
    Cs3(49, 139),
    D3(50, 147),
    Ds3(51, 156),
    E3(52, 165),
    F3(53, 175),
    Fs3(54, 185),
    G3(55, 196),
    Gs3(56, 208),
    A3(57, 220),
    As3(58, 233),
    B3(59, 247),
    //
    C4(60, 262),
    Cs4(61, 277),
    D4(62, 294),
    Ds4(63, 311),
    E4(64, 330),
    F4(65, 349),
    Fs4(66, 370),
    G4(67, 392),
    Gs4(68, 415),
    A4(69, 440),
    As4(70, 466),
    B4(71, 494),
    //
    C5(72, 523),
    Cs5(73, 554),
    D5(74, 587),
    Ds5(75, 622),
    E5(76, 659),
    F5(77, 698),
    Fs5(78, 740),
    G5(79, 784),
    Gs5(80, 831),
    A5(81, 880),
    As5(82, 932),
    B5(83, 988),
    //
    C6(84, 1046),
    Cs6(85, 1109),
    D6(86, 1175),
    Ds6(87, 1244),
    E6(88, 1318),
    F6(89, 1397),
    Fs6(90, 1480),
    G6(91, 1568),
    Gs6(92, 1661),
    A6(93, 1760),
    As6(94, 1875),
    B6(95, 1975),
    //
    C7(96, 2093),
    Cs7(97, 2217),
    D7(98, 2349),
    Ds7(99, 2489),
    E7(100, 2637),
    F7(101, 2793),
    Fs7(102, 2960),
    G7(103, 3136),
    Gs7(104, 3322),
    A7(105, 3520),
    As7(106, 3729),
    B7(107, 3951),
    //
    C8(108, 4186);

    private static Map<Integer, Note> NOTES = new TreeMap<>();
    static {
        for (Note n : Note.values()) {
            NOTES.put(n.midi, n);
        }
    }

    /** Midi note number */
    public final int midi;
    /** Note frequency in Hz */
    public final int hz;

    Note(int midi, int hz) {
        this.midi = midi;
        this.hz = hz;
    }

    public static Note get(int midi) {
        return NOTES.get(midi);
    }
}
