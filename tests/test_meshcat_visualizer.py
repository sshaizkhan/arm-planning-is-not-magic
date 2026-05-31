"""Tests for MeshcatVisualizer helpers (no real meshcat server needed)."""
import numpy as np
import pytest

meshcat = pytest.importorskip("meshcat", reason="meshcat not installed")
yourdfpy = pytest.importorskip("yourdfpy", reason="yourdfpy not installed")


def test_hex_color_white():
    from visualization.meshcat_visualizer import _rgba_to_hex
    assert _rgba_to_hex([1.0, 1.0, 1.0, 1.0]) == 0xFFFFFF


def test_hex_color_red():
    from visualization.meshcat_visualizer import _rgba_to_hex
    assert _rgba_to_hex([1.0, 0.0, 0.0, 1.0]) == 0xFF0000


def test_hex_color_blue():
    from visualization.meshcat_visualizer import _rgba_to_hex
    assert _rgba_to_hex([0.0, 0.0, 1.0, 1.0]) == 0x0000FF


def test_hex_color_mixed():
    from visualization.meshcat_visualizer import _rgba_to_hex
    result = _rgba_to_hex([0.5, 0.25, 0.0, 1.0])
    assert result == (int(0.5 * 255) << 16) | (int(0.25 * 255) << 8) | 0


def test_cylinder_alignment_matrix_shape():
    from visualization.meshcat_visualizer import _cylinder_align
    R = _cylinder_align()
    assert R.shape == (4, 4)
    assert np.allclose(R[3], [0, 0, 0, 1])


def test_cylinder_alignment_rotates_y_to_z():
    """meshcat Cylinder is Y-aligned; URDF expects Z-aligned. Rotation maps Y→Z."""
    from visualization.meshcat_visualizer import _cylinder_align
    R = _cylinder_align()
    y_axis = np.array([0, 1, 0, 0])
    rotated = R @ y_axis
    assert np.allclose(rotated[:3], [0, 0, 1], atol=1e-10)


from unittest.mock import MagicMock, patch


@pytest.fixture
def mock_viz():
    """MeshcatVisualizer with meshcat server and yourdfpy mocked out."""
    with patch('visualization.meshcat_visualizer.meshcat') as mc, \
         patch('visualization.meshcat_visualizer.yourdfpy') as yd:

        mock_server = MagicMock()
        mc.Visualizer.return_value = mock_server

        mock_robot = MagicMock()
        mock_robot.links = _make_mock_links()
        mock_robot.joints = []
        yd.URDF.load.return_value = mock_robot

        from visualization.meshcat_visualizer import MeshcatVisualizer
        viz = MeshcatVisualizer(urdf_path="fake.urdf", open_browser=False)
        yield viz, mock_server


def _make_mock_links():
    links = []

    cyl_geom = MagicMock()
    cyl_geom.box = None
    cyl_geom.sphere = None
    cyl_geom.cylinder = MagicMock(radius=0.05, length=0.2)
    cyl_geom.mesh = None

    cyl_visual = MagicMock()
    cyl_visual.geometry = cyl_geom
    cyl_visual.origin = None
    cyl_visual.material = MagicMock()
    cyl_visual.material.color = MagicMock()
    cyl_visual.material.color.rgba = [0.8, 0.2, 0.2, 1.0]

    link_a = MagicMock()
    link_a.name = "base_link"
    link_a.visuals = [cyl_visual]
    links.append(link_a)

    box_geom = MagicMock()
    box_geom.box = MagicMock(size=[0.1, 0.1, 0.4])
    box_geom.sphere = None
    box_geom.cylinder = None
    box_geom.mesh = None

    box_visual = MagicMock()
    box_visual.geometry = box_geom
    box_visual.origin = None
    box_visual.material = None

    link_b = MagicMock()
    link_b.name = "shoulder_link"
    link_b.visuals = [box_visual]
    links.append(link_b)

    return links


def test_init_sets_geometry_for_each_link(mock_viz):
    _, mock_server = mock_viz
    assert mock_server.__getitem__.call_count >= 2


def test_init_opens_no_browser_when_false(mock_viz):
    _, mock_server = mock_viz
    mock_server.open.assert_not_called()


def test_set_joint_positions_calls_set_transform(mock_viz):
    viz, mock_server = mock_viz
    q = np.array([0.5, -0.5, 0.3, -0.7, 0.2, 0.1])
    viz.set_joint_positions(q)
    assert mock_server.__getitem__.return_value.set_transform.called


def test_set_joint_positions_wrong_dof_raises(mock_viz):
    viz, _ = mock_viz
    with pytest.raises(ValueError, match="Expected 6 joints"):
        viz.set_joint_positions(np.zeros(3))


def test_visualize_configuration_calls_set_joint_positions(mock_viz):
    viz, _ = mock_viz
    viz.visualize_configuration(np.zeros(6), duration=0.0)


def test_visualize_trajectory_calls_set_joint_positions(mock_viz):
    viz, mock_server = mock_viz
    trajectory = np.zeros((5, 6))
    trajectory[:, 0] = np.linspace(0, 0.5, 5)
    viz.visualize_trajectory(trajectory, real_time=False, show_ee_trail=False)


def test_visualize_trajectory_with_trail_sets_line(mock_viz):
    viz, mock_server = mock_viz
    trajectory = np.zeros((5, 6))
    viz.visualize_trajectory(trajectory, real_time=False, show_ee_trail=True)
    calls = [str(c) for c in mock_server.__getitem__.call_args_list]
    assert any("trail" in c for c in calls)


def test_visualize_trajectory_time_stamps_shape(mock_viz):
    viz, _ = mock_viz
    trajectory = np.zeros((4, 6))
    time_stamps = np.linspace(0, 1, 4)
    viz.visualize_trajectory(trajectory, time_stamps=time_stamps, real_time=False)


def test_visualize_path_sets_line_object(mock_viz):
    viz, mock_server = mock_viz
    path = np.zeros((5, 6))
    path[:, 0] = np.linspace(0, 0.3, 5)

    def fake_fk(q):
        pos = np.array([q[0] * 0.5, 0.0, 0.5])
        return pos, np.eye(3)

    viz.visualize_path(path, fk_func=fake_fk)
    calls = [str(c) for c in mock_server.__getitem__.call_args_list]
    assert any("path" in c for c in calls)


def test_clear_path_deletes_path_subtree(mock_viz):
    viz, mock_server = mock_viz
    viz.clear_path()
    mock_server.__getitem__.return_value.delete.assert_called()


def test_visualize_trajectory_path_samples_correctly(mock_viz):
    viz, mock_server = mock_viz
    traj = np.zeros((20, 6))

    def fake_fk(q):
        return np.zeros(3), np.eye(3)

    viz.visualize_trajectory_path(traj, fk_func=fake_fk, sample_every=5)


def test_add_box_returns_path_string(mock_viz):
    viz, mock_server = mock_viz
    path = viz.add_box(center=(0.5, 0.0, 0.3), size=(0.1, 0.1, 0.1))
    assert isinstance(path, str)
    assert "box" in path


def test_add_sphere_returns_path_string(mock_viz):
    viz, mock_server = mock_viz
    path = viz.add_sphere(center=(0.3, 0.1, 0.4), radius=0.05)
    assert isinstance(path, str)
    assert "sphere" in path


def test_add_cylinder_returns_path_string(mock_viz):
    viz, mock_server = mock_viz
    path = viz.add_cylinder(center=(0.2, 0.2, 0.2), radius=0.04, height=0.3)
    assert isinstance(path, str)
    assert "cylinder" in path


def test_remove_body_calls_delete(mock_viz):
    viz, mock_server = mock_viz
    path = viz.add_box(center=(0, 0, 0), size=(0.1, 0.1, 0.1))
    viz.remove_body(path)
    mock_server.__getitem__.return_value.delete.assert_called()


def test_add_marker_returns_path_string(mock_viz):
    viz, mock_server = mock_viz
    path = viz.add_marker(position=(0.3, 0.3, 0.3))
    assert isinstance(path, str)
    assert "marker" in path


def test_add_multiple_boxes_unique_paths(mock_viz):
    viz, _ = mock_viz
    p1 = viz.add_box((0, 0, 0), (0.1, 0.1, 0.1))
    p2 = viz.add_box((1, 0, 0), (0.1, 0.1, 0.1))
    assert p1 != p2


def test_visualize_multi_planner_draws_one_line_per_planner(mock_viz):
    viz, mock_server = mock_viz

    def fake_fk(q):
        return np.array([q[0] * 0.3, 0.0, 0.4]), np.eye(3)

    planner_results = {
        "RRT": {"trajectory": np.zeros((5, 6))},
        "PRM": {"trajectory": np.ones((5, 6)) * 0.1},
        "Failed": {"trajectory": None},
    }
    result = viz.visualize_multi_planner_trajectories(
        planner_results, fk_func=fake_fk, show_labels=False
    )
    assert "RRT" in result
    assert "PRM" in result
    assert "Failed" not in result


def test_close_disconnects(mock_viz):
    viz, mock_server = mock_viz
    viz.close()


def test_context_manager(mock_viz):
    viz, mock_server = mock_viz
    with viz:
        pass
