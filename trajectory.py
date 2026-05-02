# Stephen Snelson Shanthosh Raaj Gabe Syzybalski
# ENPM661 - RO01 Spring 2026
# Project 5 - Competition with RRT
# Trajectory Data Class

from dataclasses import dataclass

@dataclass(frozen=True)
class Trajectory():
    """data class used to pass start point, end point, trajectory path, cost"""
    start_point: tuple[float, float]
    end_point: tuple[float, float]
    trajectory: list[tuple[float, float]]
    cost: float
    
        