#include "tutorial/sandBox/sandBox.h"
#include "E:\Repositories\igl_playtesting\tutorial\sandBox\collision_detect.h"
#include "igl/edge_flaps.h"
#include "igl/collapse_edge.h"
#include "Eigen/dense"
#include <functional>
#include <igl/boundary_conditions.h>
#include <igl/bbw.h>
#include <igl/normalize_row_sums.h>
#include <igl/readTGF.h>
#include <igl/readMESH.h>
#include <igl/readOBJ.h>
#include <igl/directed_edge_parents.h>
#include <igl/readDMAT.h>
#include <igl/column_to_quats.h>
#include <igl/lbs_matrix.h>
#include <igl/forward_kinematics.h>
#include <igl/deform_skeleton.h>
#include <igl/writeDMAT.h>
#include <igl/decimate.h>
#include <igl/boundary_loop.h>
#include <igl/lscm.h>
#include <igl/map_vertices_to_circle.h>
#include <igl/harmonic.h>
#include <igl/arap.h>
#include <random>
#include "igl/png/texture_from_png.h"
#include <time.h>
#include <Windows.h>
#include <playsoundapi.h>
#include <igl/copyleft/tetgen/tetrahedralize.h>
#include <igl/writeOFF.h>
using namespace Eigen;
using namespace std;

#define DEFAULT_MOVING_SPEED 0.06f
#define DEFAULT_OBJECT_MOVING_SPEED 0.02f
#define MAXIMUM_DECIMATION_FACES 100
#define MAX_GAME_OBJECTS 3
#define MAX_LEVEL_TIME 10
#define BASE_SCORE_TO_WIN 1000
#define ROTATION_RADIUS 1.0f



SandBox::SandBox()
{
	sea_green = Vector3d(70.0 / 255.0, 252.0 / 255.0, 167.0 / 255.0);
	is_snake_moving = false;
	snake_moving_speed = DEFAULT_MOVING_SPEED;
	total_game_points = 0;
	total_game_objects = 0;
	map_size = 12;
	snake_size = 4.0f;
	object_move_speed = 0.01f;
	current_level = 1;
	dirty_data_list = false;
	is_first_person_view = false;
}

void SandBox::Init(const std::string &config)
{
	init_rng();
	std::string item_name;
	std::ifstream nameFileout;
	doubleVariable = 0;
	nameFileout.open(config);
	if (!nameFileout.is_open())
	{
		std::cout << "Can't open file "<<config << std::endl;
	}
	else
	{
		int counter=0;
		while (nameFileout >> item_name)
		{
			std::cout << "opening " << item_name << std::endl;
			load_mesh_from_file(item_name);
			
			parents.push_back(-1);
			//data().set_visible(false, 1);
			if (item_name == "data/snakeuv.obj")
			{
				data().image_texture("textures/snake.jpg");
				this->pov_mesh_index = this->data_list.size() - 1;
				
				generate_decimated_mesh(item_name, data());
				data().is_game_object = true;
				//init_skinning_weights();
				//we want to set the pov to be the snake front.
				Init_fp_pov(this->pov_mesh_index); 
			}
			else if( item_name == "data/planexy.off")
			{
				data().set_uv(data().V_uv *=( 1.0f / 2.0f));
				data().image_texture("textures/bricks.jpg");
				//data().SetCenterOfRotation(Vector3d(1, 0, 0));
				data().MyRotate(Vector3d(1, 0, 0), 3*M_PI/2.0f);
				data().MyScale(Vector3d(12, 12, 0));
				data().TranslateInSystem(GetRotation(),Vector3d(0,-0.5,0));
			}
			
			counter++;	
		}


		cubemap_idx=load_cubemap_from_file("data/cube.obj","textures/skybox");
		is_cube_inplace = true;
		load_game_objects();
		nameFileout.close();
		//load_sound();
	}
}

bool SandBox::load_sound()
{
	std::string s("data/gameaudio.wav");
	std::string stemp = std::string(s.begin(), s.end());
	LPCSTR sw = stemp.c_str();
	if (PlaySound(sw, NULL, SND_LOOP|SND_ASYNC))
	{
		printf("played sound file \n");
		return true;
	}
	else
		return false;
}

bool SandBox::init_skinning_weights()
{

	igl::readMESH("data/snake1.1.mesh", V, T, F);
	//U = V;
	igl::readTGF("data/snake_joints.tgf", C, BE);
	//IOFormat CommaInitFmt(StreamPrecision, DontAlignCols, ", ", ", ", "", "", " << ", ";");
	//std::cout << C.format(CommaInitFmt) << endl;
	//// retrieve parents for forward kinematics
	igl::directed_edge_parents(BE, P);

	//// Read pose as matrix of quaternions per row
	//MatrixXd Q;
	//igl::readDMAT("data/hand-pose.dmat", Q);
	//igl::column_to_quats(Q, pose);
	//assert(pose.size() == BE.rows());

	// List of boundary indices (aka fixed value indices into VV)
	VectorXi b;
	// List of boundary conditions of each weight function
	MatrixXd bc;
	igl::boundary_conditions(V, T, C, VectorXi(), BE, MatrixXi(), b, bc);
	// compute BBW weights matrix
	igl::BBWData bbw_data;
	// only a few iterations for sake of demo
	bbw_data.active_set_params.max_iter = 8;
	bbw_data.verbosity = 2;
	if (!igl::bbw(V, T, b, bc, bbw_data, W))
	{
		return EXIT_FAILURE;
	}

	//MatrixXd Vsurf = V.topLeftCorner(F.maxCoeff()+1,V.cols());
	//MatrixXd Wsurf;
	//if(!igl::bone_heat(Vsurf,F,C,VectorXi(),BE,MatrixXi(),Wsurf))
	//{
	//  return false;
	//}
	//W.setConstant(V.rows(),Wsurf.cols(),1);
	//W.topLeftCorner(Wsurf.rows(),Wsurf.cols()) = Wsurf = Wsurf = Wsurf = Wsurf;

	// Normalize weights to sum to one
	igl::normalize_row_sums(W, W);

	//igl::readDMAT("data/mattest.dmat", W);
	//igl::lbs_matrix(V, W, M);
	//this->append_mesh();
	//this->data().clear();
	//this->data().set_mesh(U, F);
	//this->data().set_edges(C, BE, sea_green);
	//this->data().show_lines = false;
	//this->data().show_overlay_depth = false;
	//this->data().line_width = 1;
	//parents.push_back(-1);
	//data().set_visible(false, 1);
	//data().uniform_colors(Eigen::Vector3d(51.0 / 255.0, 43.0 / 255.0, 33.3 / 255.0),
	//	Eigen::Vector3d(255.0 / 255.0, 228.0 / 255.0, 58.0 / 255.0),
	//	Eigen::Vector3d(255.0 / 255.0, 235.0 / 255.0, 80.0 / 255.0));

	//snake_obj_ref = data();
	return true;
}

void SandBox::MoveInDirection(Direction d)
{
	
	if (this->game_started && !started_turning_animation)
	{
		Bezier_Direction rotation_orientation;
		bool rotation_direction = false;
		bool flag_changed_direction = false;

		switch (d)
		{
		case Direction::up:
			if (current_direction != Direction::down && current_direction != Direction::up)
			{
				if (current_direction == Direction::left)
				{
					rotation_orientation = Bezier_Direction::left_up;
					rotation_direction = false;
				}
				else
				{
					rotation_orientation = Bezier_Direction::right_up;
					rotation_direction = true;
				}
				snake_moving_translation = Vector3d(0, 0, 1);
				current_direction=Direction::up;
				flag_changed_direction = true;
			}
			break;
		case Direction::down:
			if (current_direction != Direction::up && current_direction != Direction::down)
			{
				if (current_direction == Direction::left)
				{
					rotation_orientation = Bezier_Direction::left_down;
					rotation_direction = true;
				}
				else
				{
					rotation_orientation = Bezier_Direction::right_down;
					rotation_direction = false;
				}

				snake_moving_translation = Vector3d(0, 0, -1);
				current_direction = Direction::down;
				flag_changed_direction = true;
			}
			break;
		case Direction::left:
			if (current_direction != Direction::right && current_direction != Direction::left)
			{
				if (current_direction == Direction::up)
				{
					rotation_orientation = Bezier_Direction::up_left;
					rotation_direction = true;
				}
				else
				{
					rotation_orientation = Bezier_Direction::down_left;
					rotation_direction = false;
				}
				snake_moving_translation = Vector3d(1, 0, 0);
				current_direction = Direction::left;
				flag_changed_direction = true;
				
			}
			break;
		case Direction::right:
			if (current_direction != Direction::left && current_direction!= Direction::right)
			{
				if (current_direction == Direction::up)
				{
					rotation_orientation = Bezier_Direction::up_right;
					rotation_direction = false;
				}
				else
				{
					rotation_orientation = Bezier_Direction::down_right;
					rotation_direction = true;
				}
				snake_moving_translation = Vector3d(-1, 0, 0);
				current_direction = Direction::right;
				flag_changed_direction = true;

			}
			break;
		default:
			break;
		}
		if (flag_changed_direction)
		{
			init_turning_animation(rotation_direction, rotation_orientation);
		}

	}
}


void SandBox::change_objects_direction()
{
	srand(time(0));
	double x_direction, y_direction;
	for (int i = 0; i < data_list.size(); i++)
	{
		if (data_list[i].is_game_object && i != pov_mesh_index)
		{
			x_direction = ((double)(rand() % 10)) / 10;
			y_direction = ((double)(rand() % 10)) / 10;
			data_list[i].moving_direction = Eigen::Vector3d(x_direction, 0, y_direction).normalized();
		}
		
	}
}


void SandBox::move_game_objects()
{
	for (int i = 0; i < data_list.size(); i++)
	{
		if (data_list[i].is_game_object && i != pov_mesh_index)
		{
			data_list[i].MyTranslate(data_list[i].moving_direction * this->object_move_speed,true);
		}
	}
}

void SandBox::init_rng()
{
	for (int i = (-map_size / 2); i <= (map_size / 2); i++)
		for (int j = (-map_size / 2); j <= (map_size / 2); j++)
			rng_to_point.push_back(std::pair<int, int>(i, j));
}

std::pair<int, int> SandBox::get_random_point()
{
	std::random_device rd;
	unsigned long seed = rd();

	std::mt19937 engine(seed);
	// Distribution  {0, 1, 2, 4, 5}


	legal_numbers.clear();
	bool is_valid_point;
	//we iterate through the objects array and check for intersecting boundaries, if there if we will exclude it from 
	//the number generation
	for (int i = (-map_size / 2); i <= (map_size / 2); i++)
	{
		for (int j = (-map_size / 2); j <= (map_size / 2); j++)
		{
			is_valid_point = true;
			for (int k = 0; k < data_list.size(); k++)
			{
				if (data_list[k].is_game_object)
				{
					Eigen::Vector4d current_obj_center_location = data_list[k].MakeTransd() * Eigen::Vector4d::Zero();
					double x_location = current_obj_center_location(1);
					double y_location = current_obj_center_location(3);
					int x_bound_low = floor(x_location - (snake_size / 2.0));
					int x_bound_high = floor(x_location + (snake_size / 2.0));
					int y_bound_low = floor(y_location - (snake_size / 2.0));
					int y_bound_high = floor(y_location + (snake_size / 2.0));
					if ((i > x_bound_low && i < x_bound_high) && (j > y_bound_low || j < y_bound_high))
						is_valid_point = false;
				}
			}
			if (is_valid_point)
				legal_numbers.push_back(1);
			else
				legal_numbers.push_back(0);
		}
	}

	std::discrete_distribution<> dist(legal_numbers.begin(), legal_numbers.end());
	auto rng = std::bind(dist, std::ref(engine));
	int x = rng();
	if (x >= 0 && x < rng_to_point.size())
		return std::pair<int, int>(rng_to_point[x]);
	else
	{
		return std::pair<int, int>(0, 0);
		printf("failed to generate random point! \n");
	}
}

void SandBox::Start_Level()
{
	timer = std::chrono::steady_clock::now();
	snake_moving_speed = (double)current_level * DEFAULT_MOVING_SPEED;
	is_snake_moving = true;
	game_started = true;
	total_game_points = 0;
	object_move_speed = (double)current_level * DEFAULT_OBJECT_MOVING_SPEED;
	spawn_time = 3.0f;
	score_to_finish_level = current_level * 500;
	last_time_spawned = 0;
	animation_current_time = 0.0;
	started_turning_animation = false;
	current_direction = Direction::up;
	snake_moving_translation = Vector3d(0, 0, 1);
	
}

bool SandBox::load_game_objects()
{
	return load_box_object();
}

bool SandBox::load_box_object()
{
	Eigen::MatrixXd corner_normals;
	Eigen::MatrixXi fNormIndices;

	Eigen::MatrixXd UV_V;
	Eigen::MatrixXi UV_F;
	Eigen::MatrixXd V;
	Eigen::MatrixXi F;
	Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> texture_R;
	Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> texture_G;
	Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> texture_B;
	Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> texture_A;

	if (!(
		igl::readOBJ(
			"data/cubenew1.obj",
			V, UV_V, corner_normals, F, UV_F, fNormIndices)))
	{
		return false;
	}
	//read texture from file
	if (!igl::png::texture_from_png("textures/cube2.png", texture_R, texture_G, texture_B, texture_A))
	{
		std::cout << "can't open texture file for game object" << std::endl;
		return false;
	}

	//put object data into the map.
	game_objects_map.insert(std::pair<std::string, 
							struct game_object>("cube", game_object {V,F,UV_V,UV_F,texture_R, texture_G, texture_B, texture_A })
							);
	return true;
}


int SandBox::create_viewer_data_from_object_data(std::string obj_name)
{
	auto it = game_objects_map.find(obj_name);
	if (it != game_objects_map.end())
	{
		// Create new data slot and set to selected
		if (!(data().F.rows() == 0 && data().V.rows() == 0))
		{
			append_mesh();
		}
		data().clear();
		data().set_mesh(it->second.V, it->second.F);
		if (it->second.V_uv.rows() == 0)
		{
			data().grid_texture();
		}
		else
			data().set_uv(it->second.V_uv, it->second.F_uv);

		data().compute_normals();
		data().uniform_colors(Eigen::Vector3d(150.0 / 255.0, 150.0 / 255.0, 150.0 / 255.0),
			Eigen::Vector3d(130.0 / 255.0, 130.0 / 255.0, 130.0 / 255.0),
			Eigen::Vector3d(180.0 / 255.0, 180.0 / 255.0, 130.0 / 255.0));
		data().set_texture(it->second.texture_R.transpose(), it->second.texture_G.transpose(), it->second.texture_B.transpose());
		data().dirty |= igl::opengl::MeshGL::DIRTY_TEXTURE;
		parents.push_back(-1);
		//data().set_visible(false, 1);
		data().is_game_object = true;
		dirty_data_list = true; //for renderer
		data().dirty_object = true;
		return data_list.size() - 1;
		
	}

	return -1; //failed to find object
}

bool SandBox::end_current_level(bool reason) //false = time ,true = score reached
{
	//remove all current objects
	for (int i = 0; i < data_list.size() ; i++)
	{
		if (remove_game_object(i))
			i = 0;
	}
	//stop game and snake
	is_snake_moving = false;
	game_started = false;
	//get snake current location
	Eigen::Vector3d reverse_trans_snake = -1 * (data_list[pov_mesh_index].MakeTransd() * Eigen::Vector4d(0, 0, 0, 1)).head<3>();
	if (!is_cube_inplace)
	{
		data_list[cubemap_idx].MyTranslate(reverse_trans_snake, true);
		is_cube_inplace = true;
	}
	//reset snake rotation and translation
	if (pov_mesh_index >= 0 && pov_mesh_index < data_list.size())
	{
		data_list[pov_mesh_index].MyTranslate(reverse_trans_snake, true);
		data_list[pov_mesh_index].MyRotate(data_list[pov_mesh_index].GetRotation().inverse());
	}
	if (reason) //success
	{
		printf("Level completed! \n");
		current_level++;

	}
	else //failed level
	{
		printf("Level failed... time limit reached \n");
	}
	return true;

}

void SandBox::spawn_new_object()
{
	if (this->total_game_objects < MAX_GAME_OBJECTS)
	{
		int object_index=create_viewer_data_from_object_data("cube");
		std::pair<int,int> starting_point = get_random_point();
		if (object_index >= 0 && object_index < data_list.size())
		{
			data_list[object_index].moving_direction = Eigen::Vector3d(0,0,0);
			data_list[object_index].MyScale(Eigen::Vector3d(0.5, 0.5, 0.5));
			data_list[object_index].MyTranslate(Eigen::Vector3d(starting_point.first, 0, starting_point.second), false);
			total_game_objects++;
		}
	}
}



Vector3f SandBox::GetFPdirection()
{
	if (false)
	{
		VectorXf v_transformed(4);
		v_transformed << this->pov_camera_direction , 1;
		v_transformed = (this->data_list[this->pov_mesh_index].MakeTransd().cast<float>() * v_transformed);
		return v_transformed.head<3>();
	}
	else
	{
		return Vector3f(0, 0, 1);
	}
}

/// <summary>
/// used to simplify meshes to make collision detection smoother
/// </summary>
/// <param name="mesh_name"> - name of simplified mesh to store in the dictionary </param>
/// <param name="mesh_to_add"> - ViewerData object to get mesh from </param>
bool SandBox::generate_decimated_mesh(std::string mesh_name, igl::opengl::ViewerData& mesh_to_add)
{
	auto it = decimated_mesh_map.find(mesh_name);
	if (it == decimated_mesh_map.end())
	{
		MatrixXd U; //decimated vertices
		MatrixXi G; //decimated faces
		VectorXi J;
		if (!igl::decimate(mesh_to_add.V, mesh_to_add.F, MAXIMUM_DECIMATION_FACES, U, G, J))
			return false;

		decimated_mesh_map.insert(std::pair<std::string, std::pair<Eigen::MatrixXd, Eigen::MatrixXi>>(mesh_name, std::pair<Eigen::MatrixXd, Eigen::MatrixXi>(U, G)));
		//update the tree of the mesh
		mesh_to_add.tree.init(U, G);
		return true;
	}
	else
	{
		mesh_to_add.tree.init(it->second.first, it->second.second); //get the already stored U,G matrices
		return true;
	}
}

SandBox::~SandBox()
{

}

void SandBox::continue_turning_animation()
{
	animation_current_time += 1.0;
	if (animation_current_time <= 60.5)
	{
		double t = animation_current_time / 60.0;
		Eigen::Vector3d next_point=bez_curve.P1 + (1.0 - t)* (1.0 - t) * (bez_curve.P0 - bez_curve.P1) +  t * t * (bez_curve.P2 - bez_curve.P1);
		Eigen::Vector3d trans_amount = next_point-(data_list[pov_mesh_index].MakeTransd() * Eigen::Vector4d(0, 0, 0, 1)).head<3>();
		data_list[pov_mesh_index].MyTranslate(trans_amount, true);
		if(rotation_animation_direction)
			data_list[pov_mesh_index].MyRotate(Eigen::Vector3d(0, 1, 0), (0.5*M_PI)/60.0);
		else
			data_list[pov_mesh_index].MyRotate(Eigen::Vector3d(0, 1, 0), -1.0*(0.5 * M_PI) / 60.0);
	}
	else
	{
		animation_current_time = 0.0;
		started_turning_animation = false;
	}
}

void SandBox::init_turning_animation(bool direction, Bezier_Direction bezier_location)
{
	bez_curve.P0 = (data_list[pov_mesh_index].MakeTransd() * Eigen::Vector4d(0, 0, 0, 1)).head<3>();
	switch (bezier_location)
	{
	case Bezier_Direction::right_up:
		bez_curve.P1 = bez_curve.P0 + Eigen::Vector3d(-ROTATION_RADIUS, 0, 0);
		bez_curve.P2 = bez_curve.P1 + Eigen::Vector3d(0, 0, ROTATION_RADIUS);
		break;
	case Bezier_Direction::up_right:
		bez_curve.P1 = bez_curve.P0 + Eigen::Vector3d(0, 0, ROTATION_RADIUS);
		bez_curve.P2 = bez_curve.P1 + Eigen::Vector3d(-ROTATION_RADIUS, 0, 0);
		break;
	case Bezier_Direction::left_up:
		bez_curve.P1 = bez_curve.P0 + Eigen::Vector3d(ROTATION_RADIUS, 0, 0);
		bez_curve.P2 = bez_curve.P1 + Eigen::Vector3d(0, 0, ROTATION_RADIUS);
		break;
	case Bezier_Direction::up_left:
		bez_curve.P1 = bez_curve.P0 + Eigen::Vector3d(0, 0, ROTATION_RADIUS);
		bez_curve.P2 = bez_curve.P1 + Eigen::Vector3d(ROTATION_RADIUS, 0, 0);
		break;
	case Bezier_Direction::down_left:
		bez_curve.P1 = bez_curve.P0 + Eigen::Vector3d(0, 0, -ROTATION_RADIUS);
		bez_curve.P2 = bez_curve.P1 + Eigen::Vector3d(ROTATION_RADIUS, 0, 0);
		break;
	case Bezier_Direction::left_down:
		bez_curve.P1 = bez_curve.P0 + Eigen::Vector3d(ROTATION_RADIUS, 0, 0);
		bez_curve.P2 = bez_curve.P1 + Eigen::Vector3d(0, 0, -ROTATION_RADIUS);
		break;
	case Bezier_Direction::down_right:
		bez_curve.P1 = bez_curve.P0 + Eigen::Vector3d(0, 0, -ROTATION_RADIUS);
		bez_curve.P2 = bez_curve.P1 + Eigen::Vector3d(-ROTATION_RADIUS, 0, 0);
		break;
	case Bezier_Direction::right_down:
		bez_curve.P1 = bez_curve.P0 + Eigen::Vector3d(-ROTATION_RADIUS, 0, 0);
		bez_curve.P2 = bez_curve.P1 + Eigen::Vector3d(0, 0, -ROTATION_RADIUS);
		break;

	}
	
	
	if (direction) //clockwise
	{
		rotation_animation_direction = true;
	}
	else //counter clockwise
	{
		rotation_animation_direction = false;
	}
	started_turning_animation = true;
}


bool SandBox::check_out_of_bounds()
{
	double boundary = 1.5*((double)map_size / 2.0);
	if (pov_mesh_index >= 0 && pov_mesh_index < data_list.size())
	{
		Eigen::Vector3d snake_location = (data_list[pov_mesh_index].MakeTransd() * Eigen::Vector4d(0, 0, 0, 1)).head<3>();
		if (snake_location.x() > boundary ||
			snake_location.x() < -boundary ||
			snake_location.z() > boundary ||
			snake_location.z() < -boundary)
		{
			return true;
		}
		else
			return false;
	}
	else
		return false;
}

void SandBox::translate_cubemap()
{
	Eigen::Vector3d snake_location = (data_list[pov_mesh_index].MakeTransd() * Eigen::Vector4d(0, 0, 0, 1)).head<3>();
	if (is_first_person_view)
	{
		if (cubemap_idx >= 0)
		{
			data_list[cubemap_idx].MyTranslate(snake_moving_speed * snake_moving_translation, true);
		}
		if (is_cube_inplace)
			is_cube_inplace = false;
	}
	else
	{
		if (!is_cube_inplace)
		{
			data_list[cubemap_idx].MyTranslate(-snake_location, true);
			is_cube_inplace = true;
		}
	}
}

void SandBox::Animate()
{
	if (game_started)
	{
		//check animation pending
		if (started_turning_animation)
		{
			continue_turning_animation();
		}
		else
		{
			data_list[pov_mesh_index].MyTranslate(snake_moving_speed * snake_moving_translation, true);
		}
		translate_cubemap();
		//check collision
		for (int i = 0; i < data_list.size(); i++)
		{
			if (data_list[i].is_game_object && i!=pov_mesh_index)
			{
				if (check_intersection(data_list, pov_mesh_index, i))
				{
					total_game_points += 100;
					printf("total score: %f \n", total_game_points);
					if (!remove_game_object(i))
					{
						printf("could not remove game object - mesh index %d", i);
					}
				}
			}
		}
		handle_spawner();
		move_game_objects();
		
		if (check_end_game() || check_out_of_bounds())
		{
			end_current_level(false);
		}
		else
		{
			if (this->total_game_points >= current_level * BASE_SCORE_TO_WIN)
			{
				end_current_level(true);
			}
		}
	}
}

bool SandBox::check_end_game()
{
	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
	long elapsed = std::chrono::duration_cast<std::chrono::seconds> (end - timer).count();
	if (elapsed > MAX_LEVEL_TIME)
	{
		return true;
	}
	else
	{
		return false;
	}
}

void SandBox::handle_spawner()
{
	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
	long elapsed = std::chrono::duration_cast<std::chrono::seconds> (end - timer).count();
	if (elapsed !=0 && (elapsed % (long)spawn_time == 0) && elapsed!=last_time_spawned)
	{
		spawn_new_object();
		change_objects_direction();
		last_time_spawned = elapsed;
	}
}

bool SandBox::remove_game_object(const int mesh_idx)
{
	if (mesh_idx >= 0 && mesh_idx < data_list.size() && mesh_idx != pov_mesh_index && data_list[mesh_idx].is_game_object)
	{
		data_list.erase(data_list.begin() + mesh_idx);
		selected_data_index--;
		total_game_objects--;
		return true;
	}
	else
		return false;
}

