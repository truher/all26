package org.team100.lib.music;

/** Each player plays a different note. */
public class Polyphony {
    private final Player m_players[];

    public Polyphony(Player... players) {
        m_players = players;
    }

    public void play(double... freq) {
        for (int i = 0; i < freq.length && i < m_players.length; ++i) {
            m_players[i].play(freq[i]);
        }
    }
}
