"""System geometry: vehicle reference point and plane utilities.

For now it exposes the GROUND CONTACT (reference point for classifying lane/arm and
projecting to SUMO). See `ground_contact.py` for the contract.
"""

from .ground_contact import GROUND_X, GROUND_Y, add_ground_contact, ground_contact

__all__ = ["ground_contact", "add_ground_contact", "GROUND_X", "GROUND_Y"]
