"""Match phase definitions and detection logic.

Phases partition the match into distinct situations, each with its own strategy.
The :func:`detect_phase` function extracts the current :class:`MatchPhase` from
:class:`GameControlState`, and :class:`PhasePlaybook` in :mod:`.playbook` dispatches
``assign_roles`` to the appropriate phase handler.

All 16 phases are defined here. The ``NORMAL_PLAY`` phase is the fallback when no
special situation is active.
"""

from __future__ import annotations

from enum import Enum

from ..soccer_framework import GameControlState, GameState, SetPlay


class MatchPhase(str, Enum):
    """All distinct match situations that can have independent strategy.

    Values use lowercase with underscores for logging and serialisation.
    """

    # --- Kickoff phases (during PLAYING with secondary_time > 0) ---
    OUR_KICKOFF = "our_kickoff"
    """Our team has the kickoff and ``secondary_time > 0``."""
    OPPONENT_KICKOFF = "opponent_kickoff"
    """Opponent has the kickoff and ``secondary_time > 0``."""

    # --- Drop ball ---
    DROP_BALL = "drop_ball"
    """Playing with no kicking team (``kicking_team == 255``)."""

    # --- Our set plays ---
    OUR_THROW_IN = "our_throw_in"
    OUR_GOAL_KICK = "our_goal_kick"
    OUR_CORNER_KICK = "our_corner_kick"
    OUR_FREE_KICK = "our_free_kick"
    """Direct or indirect free kick for our team."""
    OUR_PENALTY_KICK = "our_penalty_kick"

    # --- Opponent set plays ---
    OPPONENT_THROW_IN = "opponent_throw_in"
    OPPONENT_GOAL_KICK = "opponent_goal_kick"
    OPPONENT_CORNER_KICK = "opponent_corner_kick"
    OPPONENT_FREE_KICK = "opponent_free_kick"
    """Direct or indirect free kick for the opponent."""
    OPPONENT_PENALTY_KICK = "opponent_penalty_kick"

    # --- Normal open play ---
    NORMAL_PLAY = "normal_play"
    """Open play with no set play or kickoff countdown active."""


# ---------------------------------------------------------------------------
# Set-play → MatchPhase mapping helpers
# ---------------------------------------------------------------------------


def _our_restart_phase(set_play: SetPlay) -> MatchPhase:
    """Map a set play to the corresponding 'our' match phase."""
    mapping = {
        SetPlay.THROW_IN: MatchPhase.OUR_THROW_IN,
        SetPlay.GOAL_KICK: MatchPhase.OUR_GOAL_KICK,
        SetPlay.CORNER_KICK: MatchPhase.OUR_CORNER_KICK,
        SetPlay.DIRECT_FREE_KICK: MatchPhase.OUR_FREE_KICK,
        SetPlay.INDIRECT_FREE_KICK: MatchPhase.OUR_FREE_KICK,
        SetPlay.PENALTY_KICK: MatchPhase.OUR_PENALTY_KICK,
    }
    return mapping.get(set_play, MatchPhase.NORMAL_PLAY)


def _opponent_restart_phase(set_play: SetPlay) -> MatchPhase:
    """Map a set play to the corresponding 'opponent' match phase."""
    mapping = {
        SetPlay.THROW_IN: MatchPhase.OPPONENT_THROW_IN,
        SetPlay.GOAL_KICK: MatchPhase.OPPONENT_GOAL_KICK,
        SetPlay.CORNER_KICK: MatchPhase.OPPONENT_CORNER_KICK,
        SetPlay.DIRECT_FREE_KICK: MatchPhase.OPPONENT_FREE_KICK,
        SetPlay.INDIRECT_FREE_KICK: MatchPhase.OPPONENT_FREE_KICK,
        SetPlay.PENALTY_KICK: MatchPhase.OPPONENT_PENALTY_KICK,
    }
    return mapping.get(set_play, MatchPhase.NORMAL_PLAY)


# ---------------------------------------------------------------------------
# Public detection API
# ---------------------------------------------------------------------------


def detect_phase(
    team_id: int,
    game: GameControlState,
) -> MatchPhase:
    """Detect the current :class:`MatchPhase` from GameController state.

    Only meaningful during ``PLAYING`` (where role execution happens).
    Returns :attr:`MatchPhase.NORMAL_PLAY` as the safe fallback.
    """
    has_kicking = game.has_kicking_team()
    is_our_team = game.kicking_team == team_id

    # --- Kickoff phases (PLAYING + secondary_time > 0) ---
    if game.state == GameState.PLAYING and game.secondary_time > 0 and has_kicking:
        if is_our_team:
            return MatchPhase.OUR_KICKOFF
        return MatchPhase.OPPONENT_KICKOFF

    # --- Set-play phases (PLAYING + active set_play + not stopped) ---
    if game.state == GameState.PLAYING and game.set_play != SetPlay.NONE and not game.stopped:
        if is_our_team:
            return _our_restart_phase(game.set_play)
        if has_kicking:
            return _opponent_restart_phase(game.set_play)

    # --- Drop ball (PLAYING + no kicking team) ---
    if game.state == GameState.PLAYING and not has_kicking:
        return MatchPhase.DROP_BALL

    # --- Everything else ---
    return MatchPhase.NORMAL_PLAY
