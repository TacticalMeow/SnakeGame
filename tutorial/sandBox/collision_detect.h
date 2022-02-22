#pragma once
#include <eigen/Eigen>
#include <igl/AABB.h>
#include <igl/opengl/ViewerData.h>

using namespace std;
//Collision detection using AABB for two objects
//Eden Mansdorf

/// <summary>
/// Adds a new collision box to the mesh of both objects.
/// </summary>
/// <param name="box1"> box to add to mesh with index of mesh_a </param>
/// <param name="box2"> box to add to mesh with index of mesh_b </param>
/// <param name="color"> color of drawn boxes </param>
/// <param name="datalist">vector of all meshes in scene (ViewerData objects)</param>
/// <param name="mesh_a">index of first mesh in datalist</param>
/// <param name="mesh_b">index of second mesh in datalist</param>
void addBox(Eigen::AlignedBox<double, 3>& box1, 
			Eigen::AlignedBox<double, 3>& box2, 
			Eigen::RowVector3d color,
			vector<igl::opengl::ViewerData>& datalist,
			const unsigned int mesh_a,
			const unsigned int mesh_b)
{

	unsigned int current_mesh = 0;
	for (int i = 0; i < 2; i++) {
		Eigen::MatrixXd V_box(8, 3);
		Eigen::MatrixXi E_box(12, 2);
		if (i == 0) {
			V_box <<
				box1.corner(box1.TopLeftCeil)[0], box1.corner(box1.TopLeftCeil)[1], box1.corner(box1.TopLeftCeil)[2],
				box1.corner(box1.TopRightCeil)[0], box1.corner(box1.TopRightCeil)[1], box1.corner(box1.TopRightCeil)[2],
				box1.corner(box1.TopRightFloor)[0], box1.corner(box1.TopRightFloor)[1], box1.corner(box1.TopRightFloor)[2],
				box1.corner(box1.TopLeftFloor)[0], box1.corner(box1.TopLeftFloor)[1], box1.corner(box1.TopLeftFloor)[2],
				box1.corner(box1.BottomLeftCeil)[0], box1.corner(box1.BottomLeftCeil)[1], box1.corner(box1.BottomLeftCeil)[2],
				box1.corner(box1.BottomRightCeil)[0], box1.corner(box1.BottomRightCeil)[1], box1.corner(box1.BottomRightCeil)[2],
				box1.corner(box1.BottomRightFloor)[0], box1.corner(box1.BottomRightFloor)[1], box1.corner(box1.BottomRightFloor)[2],
				box1.corner(box1.BottomLeftFloor)[0], box1.corner(box1.BottomLeftFloor)[1], box1.corner(box1.BottomLeftFloor)[2];
		}
		else {
			V_box <<
				box2.corner(box2.TopLeftCeil)[0], box2.corner(box2.TopLeftCeil)[1], box2.corner(box2.TopLeftCeil)[2],
				box2.corner(box2.TopRightCeil)[0], box2.corner(box2.TopRightCeil)[1], box2.corner(box2.TopRightCeil)[2],
				box2.corner(box2.TopRightFloor)[0], box2.corner(box2.TopRightFloor)[1], box2.corner(box2.TopRightFloor)[2],
				box2.corner(box2.TopLeftFloor)[0], box2.corner(box2.TopLeftFloor)[1], box2.corner(box2.TopLeftFloor)[2],
				box2.corner(box2.BottomLeftCeil)[0], box2.corner(box2.BottomLeftCeil)[1], box2.corner(box2.BottomLeftCeil)[2],
				box2.corner(box2.BottomRightCeil)[0], box2.corner(box2.BottomRightCeil)[1], box2.corner(box2.BottomRightCeil)[2],
				box2.corner(box2.BottomRightFloor)[0], box2.corner(box2.BottomRightFloor)[1], box2.corner(box2.BottomRightFloor)[2],
				box2.corner(box2.BottomLeftFloor)[0], box2.corner(box2.BottomLeftFloor)[1], box2.corner(box2.BottomLeftFloor)[2];
		}
		E_box <<
			0, 1,
			1, 2,
			2, 3,
			3, 0,
			4, 5,
			5, 6,
			6, 7,
			7, 4,
			0, 4,
			1, 5,
			2, 6,
			7, 3;
		for (unsigned j = 0; j < E_box.rows(); ++j)
		{
			if (i == 0)
				current_mesh = mesh_a;
			else
				current_mesh = mesh_b;
			datalist[current_mesh].add_edges
			(
				V_box.row(E_box(j, 0)),
				V_box.row(E_box(j, 1)),
				color
			);
		}
	}
}


//given 2 collision boxes , we use the 15 checks to determine if there is no collision 
bool check_intersection(Eigen::AlignedBox<double, 3>& Box1, 
						Eigen::AlignedBox<double, 3>& Box2, 
						Eigen::Matrix4f& A,
						Eigen::Matrix4f& B)
{
	Eigen::Matrix4f c = A.inverse() * B;
	Eigen::Vector4f c_ = Eigen::Vector4f(Box1.center()(0), Box1.center()(1), Box1.center()(2), 1);
	c_ = A * c_;
	Eigen::Vector3d c0 = Eigen::Vector3d(c_.x(), c_.y(), c_.z());
	c_ = Eigen::Vector4f(Box2.center()(0), Box2.center()(1), Box2.center()(2), 1);
	c_ = B * c_;
	Eigen::Vector3d c1 = Eigen::Vector3d(c_.x(), c_.y(), c_.z());
	Eigen::Vector3d d = c1 - c0;
	Eigen::Vector3d Ax = Eigen::Vector3d(A(0, 0), A(1, 0), A(2, 0));
	Eigen::Vector3d Ay = Eigen::Vector3d(A(0, 1), A(1, 1), A(2, 1));
	Eigen::Vector3d Az = Eigen::Vector3d(A(0, 2), A(1, 2), A(2, 2));
	Eigen::Vector3d Bx = Eigen::Vector3d(B(0, 0), B(1, 0), B(2, 0));
	Eigen::Vector3d By = Eigen::Vector3d(B(0, 1), B(1, 1), B(2, 1));
	Eigen::Vector3d Bz = Eigen::Vector3d(B(0, 2), B(1, 2), B(2, 2));
	double a0, a1, a2, b1, b2, b0, R0, R1, R;
	a0 = (Box1.sizes()(0) / 2) + 0.0001; a1 = (Box1.sizes()(1) / 2) + 0.0001; a2 = (Box1.sizes()(2) / 2) + 0.0001;
	b0 = (Box2.sizes()(0) / 2) + 0.0001; b1 = (Box2.sizes()(1) / 2) + 0.0001; b2 = (Box2.sizes()(2) / 2) + 0.0001;
	R = std::abs(Ax.dot(d));
	R0 = a0;
	R1 = (b0 * std::abs(c(0, 0))) + (b1 * std::abs(c(0, 1))) + (b2 * std::abs(c(0, 2)));
	if (R > R1 + R0)
		return false;
	R = std::abs(Ay.dot(d));
	R0 = a1;
	R1 = (b0 * std::abs(c(1, 0))) + (b1 * std::abs(c(1, 1))) + (b2 * std::abs(c(1, 2)));
	if (R > R1 + R0)
		return false;
	R = std::abs(Az.dot(d));
	R0 = a2;
	R1 = (b0 * std::abs(c(2, 0))) + (b1 * std::abs(c(2, 1))) + (b2 * std::abs(c(2, 2)));
	if (R > R1 + R0)
		return false;
	R0 = (a0 * std::abs(c(0, 0))) + (a1 * std::abs(c(1, 0))) + (a2 * std::abs(c(2, 0)));
	R1 = b0;
	R = std::abs(Bx.dot(d));
	if (R > R1 + R0)
		return false;
	R0 = (a0 * std::abs(c(0, 1))) + (a1 * std::abs(c(1, 1))) + (a2 * std::abs(c(2, 1)));
	R1 = b1;
	R = std::abs(By.dot(d));
	if (R > R1 + R0)
		return false;
	R0 = (a0 * std::abs(c(0, 2))) + (a1 * std::abs(c(1, 2))) + (a2 * std::abs(c(2, 2)));
	R1 = b2;
	R = std::abs(Bz.dot(d));
	if (R > R1 + R0)
		return false;
	R0 = (a1 * std::abs(c(2, 0))) + (a2 * std::abs(c(1, 0)));
	R1 = (b1 * std::abs(c(0, 2))) + (b2 * std::abs(c(0, 1)));
	R = std::abs((d.dot(c(1, 0) * Az)) - (d.dot(c(2, 0) * Ay)));
	if (R > R1 + R0)
		return false;
	R0 = (a1 * std::abs(c(2, 1))) + (a2 * std::abs(c(1, 1)));
	R1 = (b0 * std::abs(c(0, 2))) + (b2 * std::abs(c(0, 0)));
	R = std::abs((d.dot(c(1, 1) * Az)) - (d.dot(c(2, 1) * Ay)));
	if (R > R1 + R0)
		return false;
	R0 = (a1 * std::abs(c(2, 2))) + (a2 * std::abs(c(1, 2)));
	R1 = (b0 * std::abs(c(0, 1))) + (b1 * std::abs(c(0, 0)));
	R = std::abs((d.dot(c(1, 2) * Az)) - (d.dot(c(2, 2) * Ay)));
	if (R > R1 + R0)
		return false;
	R0 = (a0 * std::abs(c(2, 0))) + (a2 * std::abs(c(0, 0)));
	R1 = (b1 * std::abs(c(1, 2))) + (b2 * std::abs(c(1, 1)));
	R = std::abs((d.dot(c(2, 0) * Ax)) - (d.dot(c(0, 0) * Az)));
	if (R > R1 + R0)
		return false;
	R0 = (a0 * std::abs(c(2, 1))) + (a2 * std::abs(c(0, 1)));
	R1 = (b0 * std::abs(c(1, 2))) + (b2 * std::abs(c(1, 0)));
	R = std::abs((d.dot(c(2, 1) * Ax)) - (d.dot(c(0, 1) * Az)));
	if (R > R1 + R0)
		return false;
	R0 = (a0 * std::abs(c(2, 2))) + (a2 * std::abs(c(0, 2)));
	R1 = (b0 * std::abs(c(1, 1))) + (b1 * std::abs(c(1, 0)));
	R = std::abs((d.dot(c(2, 2) * Ax)) - (d.dot(c(0, 2) * Az)));
	if (R > R1 + R0)
		return false;
	R0 = (a0 * std::abs(c(1, 0))) + (a1 * std::abs(c(0, 0)));
	R1 = (b1 * std::abs(c(2, 2))) + (b2 * std::abs(c(2, 1)));
	R = std::abs((d.dot(c(0, 0) * Ay)) - (d.dot(c(1, 0) * Ax)));
	if (R > R1 + R0)
		return false;
	R0 = (a0 * std::abs(c(1, 1))) + (a1 * std::abs(c(0, 1)));
	R1 = (b0 * std::abs(c(2, 2))) + (b2 * std::abs(c(2, 0)));
	R = std::abs((d.dot(c(0, 1) * Ay)) - (d.dot(c(1, 1) * Ax)));
	if (R > R1 + R0)
		return false;
	R0 = (a0 * std::abs(c(1, 2))) + (a1 * std::abs(c(0, 2)));
	R1 = (b0 * std::abs(c(2, 1))) + (b1 * std::abs(c(2, 0)));
	R = std::abs((d.dot(c(0, 2) * Ay)) - (d.dot(c(1, 2) * Ax)));
	if (R > R1 + R0)
		return false;
	return true;
}


/*
  Recursive function that traverses the kdtrees of the objects, if we hit both leaves , then we have collision ,
  and we add the small collision box to the objects as overlay lines
*/
bool check_rec_intersection(igl::AABB<Eigen::MatrixXd, 3>& t1,
	igl::AABB<Eigen::MatrixXd, 3>& t2,
	Eigen::Matrix4f& A,
	Eigen::Matrix4f& B,
	vector<igl::opengl::ViewerData>& datalist,
	const unsigned int mesh_a,
	const unsigned int mesh_b)
{
	bool f = check_intersection(t1.m_box, t2.m_box, A, B);
	if (!f)
	{
		return false;
	}
	if (t1.is_leaf() && t2.is_leaf())
	{
		addBox(t1.m_box, t2.m_box, Eigen::RowVector3d(1.0, 0.0, 1.0), datalist, mesh_a, mesh_b);
		return true;
	}
	if (t1.is_leaf())
	{
		datalist[mesh_a].lastTree = t1;
		return (f && (check_rec_intersection(t1, *t2.m_right, A, B, datalist, mesh_a, mesh_b) || check_rec_intersection(t1, *t2.m_left, A, B, datalist, mesh_a, mesh_b)));
	}

	if (t2.is_leaf())
	{
		datalist[mesh_b].lastTree = t2;
		return (f && (check_rec_intersection(*t1.m_right, t2, A, B, datalist, mesh_a, mesh_b) || check_rec_intersection(*t1.m_left, t2, A, B, datalist, mesh_a, mesh_b)));
	}
	datalist[mesh_b].lastTree = t2;
	datalist[mesh_a].lastTree = t1;
	if (check_rec_intersection(*t1.m_right, *t2.m_right, A, B, datalist, mesh_a, mesh_b))
		return f;
	if (check_rec_intersection(*t1.m_right, *t2.m_left, A, B, datalist, mesh_a, mesh_b))
		return f;
	if (check_rec_intersection(*t1.m_left, *t2.m_left, A, B, datalist, mesh_a, mesh_b))
		return f;
	if (check_rec_intersection(*t1.m_left, *t2.m_right, A, B, datalist, mesh_a, mesh_b))
		return f;


	return false;
}


bool check_intersection(
	vector<igl::opengl::ViewerData>& datalist,
	const unsigned int mesh_a,
	const unsigned int mesh_b)
{
	return check_rec_intersection(datalist[mesh_a].tree,
		datalist[mesh_b].tree,
		datalist[mesh_a].MakeTransScale(),
		datalist[mesh_b].MakeTransScale(), datalist, mesh_a, mesh_b);
}