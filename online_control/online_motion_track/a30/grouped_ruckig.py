"""Time synchronization within each finger, on one full-hand sample clock."""
from types import SimpleNamespace

import ruckig


def finger_groups(names):
    groups = {}
    for i, name in enumerate(names):
        finger, joint = name.split('_hand_', 1)[1].split('_', 1)
        if finger not in ('thumb', 'index', 'mid', 'ring', 'pinky') or not joint.startswith(
                ('swing_', 'rota', 'MCP_', 'PIP_', 'DIP_')):
            raise ValueError(f'Unknown A3.0 joint: {name}')
        # Keep a finger's complete kinematic chain coordinated. Splitting its
        # base rotation/splay from flexion can introduce intermediate collisions.
        groups.setdefault(finger, []).append(i)
    return groups


class GroupedOutput:
    def __init__(self, dofs):
        self.new_position = [0.] * dofs
        self.new_velocity = [0.] * dofs
        self.new_acceleration = [0.] * dofs
        self.trajectory = SimpleNamespace(position_extrema=[None] * dofs,
                                          independent_min_durations=[0.] * dofs,
                                          duration=0.)

    def pass_to_input(self, parameters):
        parameters.current_position = self.new_position
        parameters.current_velocity = self.new_velocity
        parameters.current_acceleration = self.new_acceleration


class GroupedRuckig:
    """Keep the controller's full-hand input and validation boundary intact.

    Only successful, collision-checked combined samples advance the shared input.
    A retry copies that same starting state into every group, so unaffected groups
    cannot advance twice when another finger needs a protective replan.
    """
    fields = ('current_position', 'current_velocity', 'current_acceleration',
              'target_position', 'target_velocity', 'target_acceleration',
              'max_velocity', 'max_acceleration', 'max_jerk')

    def __init__(self, names, delta_time):
        self.delta_time = delta_time
        self.groups = finger_groups(names)
        self.solvers = [(indices, ruckig.Ruckig(len(indices), delta_time),
                         ruckig.InputParameter(len(indices)), ruckig.OutputParameter(len(indices)))
                        for indices in self.groups.values()]
        self.group_durations = {}
        self._requests = {}
        self._results = {}

    def reset(self):
        self._requests.clear()
        self._results.clear()
        for _, solver, _, _ in self.solvers:
            solver.reset()

    def update(self, parameters, output):
        values = {field: getattr(parameters, field) for field in self.fields}
        result = ruckig.Result.Finished
        durations = {}
        for key, (indices, solver, group_input, group_output) in zip(self.groups, self.solvers):
            solver.delta_time = self.delta_time
            request = (self.delta_time, *(tuple(full[i] for i in indices) for full in values.values()))
            if self._requests.get(key) == request:
                # Reuse the exact candidate when another group triggered a
                # retry; numerical replanning must not perturb this group.
                status = self._results[key]
            else:
                for field, subset in zip(self.fields, request[1:]):
                    setattr(group_input, field, list(subset))
                group_input.synchronization = ruckig.Synchronization.Time
                status = solver.update(group_input, group_output)
                if status in (ruckig.Result.Working, ruckig.Result.Finished):
                    self._requests[key] = request
                    self._results[key] = status
            if status not in (ruckig.Result.Working, ruckig.Result.Finished):
                return status
            if status == ruckig.Result.Working:
                result = status
            extrema = group_output.trajectory.position_extrema
            independent = group_output.trajectory.independent_min_durations
            at_rest = (status == ruckig.Result.Finished
                       and not any(group_input.target_velocity)
                       and not any(group_input.target_acceleration))
            for local, global_index in enumerate(indices):
                # Finished zero-derivative targets are exact resting states.
                # Residual 1e-16 derivatives otherwise trigger zero-duration
                # replanning after a boundary position was normalized.
                output.new_position[global_index] = (group_input.target_position[local] if at_rest
                                                     else group_output.new_position[local])
                output.new_velocity[global_index] = 0. if at_rest else group_output.new_velocity[local]
                output.new_acceleration[global_index] = 0. if at_rest else group_output.new_acceleration[local]
                output.trajectory.position_extrema[global_index] = extrema[local]
                output.trajectory.independent_min_durations[global_index] = independent[local]
            durations[key] = group_output.trajectory.duration
        self.group_durations = durations
        output.trajectory.duration = max(durations.values(), default=0.)
        return result
