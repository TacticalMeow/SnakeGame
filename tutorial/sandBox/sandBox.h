#pragma once
#include "igl/opengl/glfw/Viewer.h"
#include "igl/aabb.h"
#include <chrono>

typedef
std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond> >
RotationList;

enum class Direction { up, down, left,right };
enum class Bezier_Direction { up_left,up_right, down_left,down_right, left_up,left_down ,right_up,right_down };

//we store here an objects data, for using it multiple times without having to read it from file everytime
struct game_object { Eigen::MatrixXd V; 
					 Eigen::MatrixXi F; 
				     Eigen::MatrixXd V_uv;
					 Eigen::MatrixXi F_uv;
					 Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> texture_R;
					 Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> texture_G;
					 Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> texture_B;
					 Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> texture_A;
					};

struct bezier_curve {
	Eigen::Vector3d P0;
	Eigen::Vector3d P1;
	Eigen::Vector3d P2;
};

class SandBox : public igl::opengl::glfw::Viewer
{
public:
	SandBox();
	~SandBox();
	void Init(const std::string& config);
	double doubleVariable;
	bool init_skinning_weights();
	int selected = 0;

	//gameplay
	int current_level;
	bool game_started=false;
	void MoveInDirection(Direction d);
	Eigen::Vector3d snake_moving_translation; //determines direction of player movement
	bool is_snake_moving;
	double snake_moving_speed; //player moving speed
	double total_game_points; //tracks total score
	int total_game_objects; //counts live current game objects
	int score_to_finish_level;
	int map_size; //map size (scalewise)
	double snake_size; //snake size
	double object_move_speed; //determines obstacles move speed
	std::map<std::string, game_object> game_objects_map;
	bool remove_game_object(const int mesh_idx);
	void handle_spawner();
	void spawn_new_object();
	bool check_end_game();
	bool end_current_level(bool reason);
	std::chrono::steady_clock::time_point timer; //game timer
	double spawn_time;
	void Start_Level();
	long last_time_spawned = 0;
	bool load_sound();
	bool is_first_person_view;
	Eigen::Vector3f GetFPdirection();
	Direction current_direction;
	bool check_out_of_bounds();
	int cubemap_idx;
	void translate_cubemap();
	bool is_cube_inplace;

	//animation related
	bezier_curve bez_curve;
	void continue_turning_animation();
	void init_turning_animation(bool direction,Bezier_Direction bezier_location);
	bool started_turning_animation = false;
	double animation_current_time;
	bool rotation_animation_direction;
	



	//rng
	std::vector<int> legal_numbers; // rng related
	std::vector<std::pair<int, int>> rng_to_point;
	std::pair<int, int> get_random_point();
	void change_objects_direction();
	void move_game_objects();
	void init_rng();

	int GetTimeLeft();
	bool load_game_objects(); //loads the items object file into memory
	bool load_box_object();
	int create_viewer_data_from_object_data(std::string obj_name);
	bool dirty_data_list;

	


	//collision related

	std::map <std::string, std::pair<Eigen::MatrixXd, Eigen::MatrixXi>> decimated_mesh_map;
	bool generate_decimated_mesh(std::string mesh_name, igl::opengl::ViewerData& mesh_to_add);

	//skinning
	Eigen::RowVector3d sea_green;
	Eigen::MatrixXd V, W, U, C, M;
	Eigen::MatrixXi T, F, BE;
	Eigen::VectorXi P;
	double anim_t = 1.0;
	double anim_t_dir = -0.03;
	RotationList pose;



private:
	void Animate();
};

