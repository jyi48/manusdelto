#!/usr/bin/env python3
"""Retargeter registry.

``build_retargeters`` returns a dict of available strategies keyed by name.
Each strategy declares its own optional dependencies; if an import fails the
strategy is simply omitted and the node falls back to 'ergo'.

'ergo' (calibratable, EMA-smoothed) is the default selectable mode and also
the seed source for 'ik' (CLIK initial guess + fallback). heuristic.py is
kept in the repo only as a historical record — nothing here imports it.
"""
from .base import Retargeter
from .ergo import ErgoRetargeter, DEFAULT_JOINT_CALIB

__all__ = ["Retargeter", "ErgoRetargeter", "DEFAULT_JOINT_CALIB", "build_retargeters"]


def build_retargeters(*, kin, ik_params, dex_params, joint_calib, logger):
    """Build every retargeter whose dependencies are satisfied.

    Always includes 'ergo' (default selectable mode, also ik's seed). Adds
    'ik' when pinocchio (and a loaded DG5F model) is available, and the two
    dex variants ('dex' = DexPilot, 'dex_vector' = vector optimizer) when the
    dex-retargeting library is installed. Missing-dependency strategies are
    simply omitted.
    """
    ergo = ErgoRetargeter(joint_calib)
    retargeters = {"ergo": ergo}

    try:
        from .ik import IKRetargeter

        retargeters["ik"] = IKRetargeter(
            kin, ergo, logger=logger, **ik_params
        )
    except ImportError as e:
        logger.warn(f"IK retargeter unavailable ({e}) — skipping")

    try:
        from .dex import DexRetargeter

        # Both dex optimizers are built up front so the GUI can switch between
        # them at runtime like any other retarget mode.
        retargeters["dex"] = DexRetargeter(
            logger=logger, optimizer="dexpilot", **dex_params
        )
        try:
            retargeters["dex_vector"] = DexRetargeter(
                logger=logger, optimizer="vector", **dex_params
            )
        except Exception as e:
            logger.warn(f"dex vector optimizer unavailable ({e}) — skipping")
    except ImportError as e:
        logger.warn(f"dex retargeter unavailable ({e}) — skipping")
    except Exception as e:
        logger.warn(f"dex retargeter failed to init ({e}) — skipping")

    return retargeters
