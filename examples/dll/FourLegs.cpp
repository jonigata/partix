// 2008/10/10 Naoyuki Hirayama

#include "FourLegs.h"
#include "PartixUser.hpp"

namespace {

class DefaultDataProvider : public DataProvider {
public:
    DefaultDataProvider(){}
    ~DefaultDataProvider(){}

    float	GetTotalGripCoefficient() { return float(15.0); }
    float	GetFrontGripCoefficient() { return float(30.0); }
    float	GetRearGripCoefficient() { return float(30.0); }
    float	GetSensoryBalanceSpeed() { return float(0.02); }
    float	GetSensoryBalanceMax() { return float(15); }
    float	GetSensoryBalanceDecrease() { return float(0.9); }
    float	GetBalanceAngleMax() { return float(720.0); }
    float	GetTurningAngleMax() { return float(5400.0); }
    float	GetBackbendAngleFactor() { return float(0.15); }
    float	GetBankRatio() { return float(0.3); }
    float	GetBalanceRatio() { return float(0.1); }
    float	GetTurningRatio() { return float(0.2); }
    float	GetBackbendRatio() { return float(0.2); }
    float	GetBrakeAngle() { return float(150.0); }
    float	GetFrictionFactor() { return float(1.3); }
    float	GetAccelFactor() { return float(9.0); }
    float	GetTurboThreshold() { return float(25.0); }
    float	GetTurboMultiplier() { return float(4.0); }
};

class DBDataProvider : public DataProvider {
public:
    DBDataProvider()
    {
        // データベース作成
        flDB::Database* iDatabase = flDB::CreateDatabase( "Manipulate" );
	
        // データベースに Attribute (情報フィールド)を追加
        iDatabase->AddAttr( "TotalGripCoefficient", flDB::TYPE_float );
        iDatabase->AddAttr( "FrontGripCoefficient", flDB::TYPE_float );
        iDatabase->AddAttr( "RearGripCoefficient", flDB::TYPE_float );
        iDatabase->AddAttr( "SensoryBalanceSpeed", flDB::TYPE_float );
        iDatabase->AddAttr( "SensoryBalanceMax", flDB::TYPE_float );
        iDatabase->AddAttr( "SensoryBalanceDecrease", flDB::TYPE_float );
        iDatabase->AddAttr( "BalanceAngleMax", flDB::TYPE_float );
        iDatabase->AddAttr( "TurningAngleMax", flDB::TYPE_float );
        iDatabase->AddAttr( "BackbendAngleFactor", flDB::TYPE_float );
        iDatabase->AddAttr( "BankRatio", flDB::TYPE_float );
        iDatabase->AddAttr( "BalanceRatio", flDB::TYPE_float );
        iDatabase->AddAttr( "TurningRatio", flDB::TYPE_float );
        iDatabase->AddAttr( "BackbendRatio", flDB::TYPE_float );
        iDatabase->AddAttr( "BrakeAngle", flDB::TYPE_float );
        iDatabase->AddAttr( "FrictionFactor", flDB::TYPE_float );
        iDatabase->AddAttr( "AccelFactor", flDB::TYPE_float );
        iDatabase->AddAttr( "TurboThreshold", flDB::TYPE_float );
        iDatabase->AddAttr( "TurboMultiplier", flDB::TYPE_float );
	
        total_grip_coefficient_ =
            iDatabase->GetAttrIndex( "TotalGripCoefficient" );
        front_grip_coefficient_ = 
            iDatabase->GetAttrIndex( "FrontGripCoefficient" );
        rear_grip_coefficient_ = 
            iDatabase->GetAttrIndex( "RearGripCoefficient" );
        sensory_balance_speed_ = 
            iDatabase->GetAttrIndex( "SensoryBalanceSpeed" );
        sensory_balance_max_ = 
            iDatabase->GetAttrIndex( "SensoryBalanceMax" );
        sensory_balance_decrease_ = 
            iDatabase->GetAttrIndex( "SensoryBalanceDecrease" );
        balance_angle_max_ = 
            iDatabase->GetAttrIndex( "BalanceAngleMax" );
        turning_angle_max_ = 
            iDatabase->GetAttrIndex( "TurningAngleMax" );
        backbend_angle_factor_ = 
            iDatabase->GetAttrIndex( "BackbendAngleFactor" );
        bank_ratio_ = 
            iDatabase->GetAttrIndex( "BankRatio" );
        balance_ratio_ = 
            iDatabase->GetAttrIndex( "BalanceRatio" );
        turning_ratio_ = 
            iDatabase->GetAttrIndex( "TurningRatio" );
        backbend_ratio_ = 
            iDatabase->GetAttrIndex( "BackbendRatio" );
        brake_angle_ = 
            iDatabase->GetAttrIndex( "BrakeAngle" );
        friction_factor_ = 
            iDatabase->GetAttrIndex( "FrictionFactor" );
        accel_factor_ = 
            iDatabase->GetAttrIndex( "AccelFactor" );
        turbo_threshold_ = 
            iDatabase->GetAttrIndex( "TurboThreshold" );
        turbo_multiplier_ = 
            iDatabase->GetAttrIndex( "TurboMultiplier" );

        // データのインスタンスを作成
        iData = iDatabase->CreateData( "Primary" );

        DefaultDataProvider ddp;
        iData->Set<float>(
            total_grip_coefficient_, ddp.GetTotalGripCoefficient() );
        iData->Set<float>(
            front_grip_coefficient_, ddp.GetFrontGripCoefficient() );
        iData->Set<float>(
            rear_grip_coefficient_, ddp.GetRearGripCoefficient() );
        iData->Set<float>(
            sensory_balance_speed_, ddp.GetSensoryBalanceSpeed() );
        iData->Set<float>(
            sensory_balance_max_, ddp.GetSensoryBalanceMax() );
        iData->Set<float>(
            sensory_balance_decrease_, ddp.GetSensoryBalanceDecrease() );
        iData->Set<float>(
            balance_angle_max_, ddp.GetBalanceAngleMax() );
        iData->Set<float>(
            turning_angle_max_, ddp.GetTurningAngleMax() );
        iData->Set<float>(
            backbend_angle_factor_, ddp.GetBackbendAngleFactor() );
        iData->Set<float>(
            bank_ratio_, ddp.GetBankRatio() );
        iData->Set<float>(
            balance_ratio_, ddp.GetBalanceRatio() );
        iData->Set<float>(
            turning_ratio_, ddp.GetTurningRatio() );
        iData->Set<float>(
            backbend_ratio_, ddp.GetBackbendRatio() );
        iData->Set<float>(
            brake_angle_, ddp.GetBrakeAngle() );
        iData->Set<float>(
            friction_factor_, ddp.GetFrictionFactor() );
        iData->Set<float>(
            accel_factor_, ddp.GetAccelFactor() );
        iData->Set<float>(
            turbo_threshold_, ddp.GetTurboThreshold() );
        iData->Set<float>(
            turbo_multiplier_, ddp.GetTurboMultiplier() );
    }
    ~DBDataProvider()
    {
        flDB::Database* iDatabase = flDB::GetDatabase( "Manipulate" );
        iDatabase->DeleteData( "Primary" );

        // データベースの削除
        flDB::DeleteDatabase( "Manipulate" );
    }

private:	
    float	GetTotalGripCoefficient()
    {
        return iData->Get<float>( total_grip_coefficient_ );
    }
    float	GetFrontGripCoefficient()
    {
        return iData->Get<float>( front_grip_coefficient_ );
    }
    float	GetRearGripCoefficient()
    {
        return iData->Get<float>( rear_grip_coefficient_ );
    }
    float	GetSensoryBalanceSpeed()
    {
        return iData->Get<float>( sensory_balance_speed_ );
    }
    float	GetSensoryBalanceMax()
    {
        return iData->Get<float>( sensory_balance_max_ );
    }
    float	GetSensoryBalanceDecrease()
    {
        return iData->Get<float>( sensory_balance_decrease_ );
    }
    float	GetBalanceAngleMax()
    {
        return iData->Get<float>( balance_angle_max_ );
    }
    float	GetTurningAngleMax()
    {
        return iData->Get<float>( turning_angle_max_ );
    }
    float	GetBackbendAngleFactor()
    {
        return iData->Get<float>( backbend_angle_factor_ );
    }
    float	GetBankRatio()
    {
        return iData->Get<float>( bank_ratio_ );
    }
    float	GetBalanceRatio()
    {
        return iData->Get<float>( balance_ratio_ );
    }
    float	GetTurningRatio()
    {
        return iData->Get<float>( turning_ratio_ );
    }
    float	GetBackbendRatio()
    {
        return iData->Get<float>( backbend_ratio_ );
    }
    float	GetBrakeAngle()
    {
        return iData->Get<float>( brake_angle_ );
    }
    float	GetFrictionFactor()
    {
        return iData->Get<float>( friction_factor_ );
    }
    float	GetAccelFactor()
    {
        return iData->Get<float>( accel_factor_ );
    }
    float	GetTurboThreshold()
    {
        return iData->Get<float>( turbo_threshold_ );
    }
    float	GetTurboMultiplier()
    {
        return iData->Get<float>( turbo_multiplier_ );
    }

private:
    flDB::Data* iData;
    int	total_grip_coefficient_;
    int	front_grip_coefficient_;
    int	rear_grip_coefficient_ ;
    int	sensory_balance_speed_;
    int	sensory_balance_max_;
    int	sensory_balance_decrease_;
    int	balance_angle_max_;
    int	turning_angle_max_;
    int	backbend_angle_factor_;
    int	bank_ratio_;
    int	balance_ratio_;
    int	turning_ratio_;
    int	backbend_ratio_;
    int	brake_angle_;
    int friction_factor_;
    int accel_factor_;
    int turbo_threshold_;
    int turbo_multiplier_;

};

struct AnalyzeData {
    Vector  control_accel;
    float   control_power;
    float   total_grip;
    float   front_grip;
    float   rear_grip;
    float   left_grip;
    float   right_grip;
    float   bottom_grip;
    Vector  curr_velocity;
    Vector  prev_velocity;
    float   speed;
    float   front_speed;
    Matrix  com;
    Matrix  pom;
    Vector  right;
    Vector  back;
    Vector  front;
    Vector  old_right;
    Vector  old_back;
    Vector  old_front;

    Vector  bbmin;
    Vector  bbmax;
    float   mass;
};

inline void OutputDebugStringA(const char*) {}
inline void DebugBreak() {}

inline bool isnan( float f ) { return f != f; }

inline
void check_quaternion(const Quaternion& q, const char* label) {
    if (isnan(q.w) || isnan(q.x) || isnan(q.y) || isnan(q.z)) {
        OutputDebugStringA("check_quaternion failed:");
        if (label) {
            OutputDebugStringA(label);
            OutputDebugStringA("\n");
        }
        DebugBreak();
    }
}

inline
void check_vector(const Vector& q) {
    if (isnan(q.x)|| isnan(q.y)|| isnan(q.z)) {
        DebugBreak();
    }
}

void normalize_orientation_matrix(Matrix& m) {
    normalize_f(m.xaxis());
    normalize_f(m.yaxis());
    normalize_f(m.zaxis());
    m.m41 = m.m42 = m.m43 = 0;
    m.m14 = m.m24 = m.m34 = 0;
    m.m44 = 1.0f;
}

void quat_to_axis_angle(const Quaternion* q, Vector* axis, float* angle) {
    //flASSERT(1.0e-6f <q->x*q->x + q->y*q->y + q->z*q->z);

    float sin_a2 = sqrtf(q->x*q->x + q->y*q->y + q->z*q->z);
    *angle = 2.0f * atan2f(sin_a2, q->w);
    axis->x = q->x;
    axis->y = q->y;
    axis->z = q->z;
}

void quat_from_axis_angle(const Vector* axis, float angle, Quaternion* q) {
    q->x = axis->x;
    q->y = axis->y;
    q->z = axis->z;
    //assert(1.0e-6f < q->x*q->x + q->y*q->y + q->z*q->z);
    float n = sqrt(q->x*q->x + q->y*q->y + q->z*q->z);
    //assert(float(0.9)<n);

    if (n < 1.0e-6f) {
        q->x = 0;
        q->y = 0;
        q->z = 0;
        q->w = 1.0f;
    } else {
        float s = float(sinf(float(0.5)* angle)/ n);
        q->x *= s;
        q->y *= s;
        q->z *= s;
        q->w = cosf(float(0.5)* angle);
    }
}

float clamp(float t, float mn, float mx) {
    if (t < -1.0f) { t = -1.0f; }
    if (1.0f < t) { t = 1.0f; }
    return t;
}

float get_axisangle(const Vector& v0, const Vector& v1, const Vector& axis) {
    if (1.0f-1.0e-6f < fabs(dot(v0, axis))) { return 0; }
    if (1.0f-1.0e-6f < fabs(dot(v1, axis))) { return 0; }

    Vector v0c; v0c = cross(v0, axis); normalize_f(v0c);
    Vector v1c; v1c = cross(v1, axis); normalize_f(v1c);

    float dot = clamp(dot(v0c, v1c), -1.0f, 1.0f);
    float angle = acosf(dot);
    if (dot(v0c, v1) < 0) {
        angle *= -1;
    }
    return angle;
}

void analyze(
    DataProvider*			dp,
    PartixUser::softvolume_type*	sv,
    float				dt,
    const Vector&			user_accel,
    const Vector&			velocity,
    const Matrix&			prev_orientaion,
    const Matrix&			curr_orientaion,
    AnalyzeData&			ad,
    float&				sensory_balance) {
    
// ユーザーコントロール
    check_vector(user_accel);
    ad.control_accel = user_accel;
    ad.control_power = ad.control_accel.Length();

    // 姿勢・スピード
    ad.pom = prev_orientaion; normalize_orientation_matrix(ad.pom);
    ad.com = curr_orientaion; normalize_orientation_matrix(ad.com);
    ad.right = ad.com.xaxis();
    ad.back  = ad.com.yaxis();
    ad.front = ad.com.zaxis();
    ad.old_right = ad.pom.xaxis();
    ad.old_back  = ad.pom.yaxis();
    ad.old_front = ad.pom.zaxis();
    ad.prev_velocity = velocity;
    ad.curr_velocity =
        (*curr_orientaion.zaxis() - *prev_orientaion.zaxis()) / dt;
    ad.front_speed = dot(ad.curr_velocity, ad.front);

    ad.speed = (velocity + (ad.curr_velocity - velocity) * 0.2f).length();

#if 0
    flMESSAGE( "#x-axis %f, %f, %f",
               curr_orientaion.m3GetVect(0)->x,
               curr_orientaion.m3GetVect(0)->y,
               curr_orientaion.m3GetVect(0)->z );
    flMESSAGE( "#y-axis %f, %f, %f",
               curr_orientaion.m3GetVect(1)->x,
               curr_orientaion.m3GetVect(1)->y,
               curr_orientaion.m3GetVect(1)->z );
    flMESSAGE( "#z-axis %f, %f, %f",
               curr_orientaion.m3GetVect(2)->x,
               curr_orientaion.m3GetVect(2)->y,
               curr_orientaion.m3GetVect(2)->z );
    flMESSAGE( "x-axis %f, %f, %f", ad.right.x, ad.right.y, ad.right.z );
    flMESSAGE( "y-axis %f, %f, %f", ad.back.x, ad.back.y, ad.back.z );
    flMESSAGE( "z-axis %f, %f, %f", ad.front.x, ad.front.y, ad.front.z );
#endif

    // grip
    float totalcoef = dp->GetTotalGripCoefficient();
    float frontcoef = dp->GetFrontGripCoefficient();
    float rearcoef = dp->GetRearGripCoefficient();
    ad.total_grip = 0;
    ad.front_grip = 0;
    ad.rear_grip = 0;
    ad.left_grip = 0;
    ad.right_grip = 0;

    ad.mass = 0;
    float fmax = std::numeric_limits<float>::max();
    ad.bbmin = Vector(fmax, fmax, fmax);
    ad.bbmax = flVect(-fmax, -fmax, -fmax);
	
    PartixUser::mesh_type*          mesh     = sv->get_mesh();
    const PartixUser::points_type&  vertices = mesh->get_points();
    for(const auto& p: vertices) {
        if (p.new_position.x < ad.bbmin.x) { ad.bbmin.x = p.new_position.x; }
        if (p.new_position.y < ad.bbmin.y) { ad.bbmin.y = p.new_position.y; }
        if (p.new_position.z < ad.bbmin.z) { ad.bbmin.z = p.new_position.z; }
        if (ad.bbmax.x < p.new_position.x) { ad.bbmax.x = p.new_position.x; }
        if (ad.bbmax.y < p.new_position.y) { ad.bbmax.y = p.new_position.y; }
        if (ad.bbmax.z < p.new_position.z) { ad.bbmax.z = p.new_position.z; }
        ad.mass += p.mass;

        //float pushout = ( p.pushout0 + p.pushout1 ).length() * p.mass;
#if 1
        Vector pushout =
            p.constraint_pushout +
            p.active_contact_pushout +
            p.passive_contact_pushout;
#else
        Vector pushout = p.pushout0 + p.pushout1;
#endif
        float length = float(pushout.length());
        float depth = float(dot(pushout, ad.back));
        
        ad.total_grip += length * p.mass;
        ad.front_grip += depth * p.mass * p.load.front_grip;
        ad.rear_grip += depth * p.mass * p.load.rear_grip;
        ad.left_grip += depth * p.mass * p.load.left_grip;
        ad.right_grip += depth * p.mass * p.load.right_grip;
    }

    ad.front_grip *= frontcoef;
    ad.rear_grip  *= rearcoef;
    ad.total_grip *= totalcoef;
    if (1.0f < ad.front_grip) { ad.front_grip = 1.0f; }
    if (1.0f < ad.rear_grip) { ad.rear_grip = 1.0f; }
    if (1.0f < ad.total_grip) { ad.total_grip = 1.0f; }

    ad.bottom_grip = (ad.front_grip + ad.rear_grip) * 0.5f;
	
    if (0 == ad.left_grip && 0 < ad.right_grip) {
        sensory_balance += dp->GetSensoryBalanceSpeed();
    } else if (0 < ad.left_grip && 0 == ad.right_grip) {
        sensory_balance -= dp->GetSensoryBalanceSpeed();
    } else {
        sensory_balance *= dp->GetSensoryBalanceDecrease();
    }

    float sb_max = DEG2RAD(dp->GetSensoryBalanceMax());
    sensory_balance = clamp(sensory_balance, -sb_max, sb_max);
}

void get_physical_rotation(
    const Matrix& pm,
    const Matrix& cm,
    Quaternion&   q0,
    Vector&  axis,
    float&   angle) {
    Quaternion pq; pm.GetQuaternion(&pq); pq.normalize();
    Quaternion cq; cm.GetQuaternion(&cq); cq.normalize();

    Quaternion ipq = pq.inverse();
    q0 = cq;
    q0 *= ipq;
    normalize_f(q0);

    quat_to_axis_angle(&q0, &axis, &angle);
    if (PARTIX_PI < angle) {
        angle = 2 * PARTIX_PI - angle;
    }
}

bool get_balance_rotation(
	const Matrix& pm,
	const Matrix& cm,
	Vector& axis,
	float&	dot,
        float	sensory_balance) {
    Vector up(float(0), float(1), float(0));   // 反重力方向ベクトル

    Vector pyv = *pm.yaxis(); // 四足歩行動物の背中, upward
    Vector pzv = *pm.zaxis(); // 四足歩行動物の顔, forward

    float up_dot_pzv = dot(up, pzv);
    if (pyv.Length() < 1.0e-6f ||
        pzv.Length() < 1.0e-6f ||
        0.6f < abs(up_dot_pzv)) {
        // 行列が腐っているとき、
        // 顔が上(下)を向いているときは補正しない
        axis = pm.xaxis();
        dot = up_dot_pzv;
        if (up_dot_pzv < 0) { dot = 1.0f; }
        return false;
    }

    // upをm_sensory_balanceで
    Quaternion q;
    quat_from_axis_angle(&pzv, sensory_balance, &q);
    flMESSAGE( "sensory_balance %f", sensory_balance );
    check_quaternion(q,"balance_rotation:from_axis_angle");

    Quaternion qb(0, up.x, up.y, up.z);
    Quaternion qc = q.inverse();
    q *= qb;
    q *= qc;
    up = Vector(q.x, q.y, q.z);
    flMESSAGE("up %f, %f, %f", up.x, up.y, up.z);
                        
    // 主計算
    Vector gzv = pzv;
    Vector gxv = cross(up, gzv);	// 目的右手方向ベクトル
    Vector gyv = cross(gzv, gxv);	// 目的背中方向ベクトル
    normalize_f(gxv);
    normalize_f(gyv);
    check_vector(gxv);
    check_vector(gyv);

    dot = dot(pyv, gyv);
    if(1.0f-1.0e-6f < abs(dot)){
        axis = pzv;
        dot = 1.0f;
    } else {
        axis = cross(pyv, gyv);
        normalize_f(axis);
    }
    return true;
}

bool get_turning_rotation(
    const Matrix& pm,
    const Matrix& cm,
    const Vector& accel,
    Vector& axis,
    float& dot) {
    check_vector(accel);

    Vector pyv = *pm.m3GetVect(1); // 四足歩行動物の背中, upward
    Vector pzv = *pm.m3GetVect(2); // 四足歩行動物の顔, forward

    Vector gzv = pzv;
    if (pyv.Length() < 1.0e-6f ||
        pzv.Length() < 1.0e-6f ||
        accel.Length()< 0.01f) {
        return false;
    }
        
    Vector ua = accel;
    ua.Normalize();
    check_vector(ua);
    Vector v0(ua.x, 0, ua.z);      
    Vector v1(gzv.x, 0, gzv.z);
    if (v0.Length() < 1.0e-6f ||
        v1.Length() < 1.0e-6f) {
        return false;
    }

    v0.Normalize();
    v1.Normalize();
    check_vector(v0);
    check_vector(v1);
    dot = v1.Dot( &v0 );
    if (1.0f-1.0e-6f < fabsf(dot)) {
        return false;
    }
    axis.Cross(&v1, &v0);
    if (axis.Length() < 1.0e-6f) {
        return false;
    }

    axis.Normalize();
    check_vector(axis);

    return true;
}

float get_bank(const Vector& v0, const Vector& z0)
{
	Vector up(0, float(1), 0);   // 反重力方向ベクトル

	Vector v(v0); v.Normalize();
	Vector z(z0); z.Normalize();

	float v_dot_z = v.Dot(&z);

	if (1.0f-1.0e-6f < v_dot_z) { return 0; } // 曲がってない
	if (v_dot_z < 0.1f) { return 0; } // 曲がりすぎ

	float theta = acosf(v_dot_z);
	theta = flMath::float_PI() * 0.5f - theta;
	float r = z0.Length() / cosf(theta); // 回転半径

	float bank = atanf(v0.Magnitude() / (r * 9.8f));

	Vector x; x.Cross(&v, &up);
	if( x.Dot(&z) < 0 ) { bank *= -1; }
        
	return bank;
}

void setup_physie(
	PartixUser::softvolume_type*	sv,
	float							xstability,
	float							ystability,
	float							zstability,
	float							mass,
	float							scale_factor)
{
	mass *= scale_factor;

	xstability *= mass;
	ystability *= mass;
	zstability *= mass;

	const float float_max = flMath::float_floatMAX();

	// bounding box
	PartixUser::mesh_type*   mesh   = sv->get_mesh();
	PartixUser::points_type& points = mesh->get_points();
	Vector bbmin(  float_max,  float_max,  float_max );
	Vector bbmax( -float_max, -float_max, -float_max );
	for( PartixUser::points_type::iterator i = points.begin() ;
		 i != points.end() ;
		 ++i ) {
		PartixUser::point_type& p = *i;
		if( p.source_position.x < bbmin.x ) { bbmin.x = p.source_position.x; }
		if( p.source_position.y < bbmin.y ) { bbmin.y = p.source_position.y; }
		if( p.source_position.z < bbmin.z ) { bbmin.z = p.source_position.z; }
		if( bbmax.x < p.source_position.x ) { bbmax.x = p.source_position.x; }
		if( bbmax.y < p.source_position.y ) { bbmax.y = p.source_position.y; }
		if( bbmax.z < p.source_position.z ) { bbmax.z = p.source_position.z; }
	}
	Vector bbw = bbmax - bbmin;

	for( PartixUser::points_type::iterator i = points.begin() ;
		 i != points.end() ;
		 ++i ) {
		PartixUser::point_type& p = *i;

		p.mass = mass;
		if( 0 < bbw.x && xstability < 0 ) { p.mass += -xstability; }
		if( 0 < bbw.y && ystability < 0 ) { p.mass += -ystability; }
		if( 0 < bbw.z && zstability < 0 ) { p.mass += -zstability; }

		Vector* v = &p.source_position;

		float xr = ( v->x - bbmin.x ) / bbw.x;
		float yr = ( v->y - bbmin.y ) / bbw.y;
		float zr = ( v->z - bbmin.z ) / bbw.z;

		if( 0 < bbw.x ) { p.mass += xr * xstability; }
		if( 0 < bbw.y ) { p.mass += yr * ystability; }
		if( 0 < bbw.z ) { p.mass += zr * zstability; }
                        
		// グリップは足元のみ
		float x = v->x;
		float y = v->y;
		float z = v->z; // + m_actor->base_offset().y;
		p.load.front_grip       = 0.0f <  z && y < bbw.y * 0.02f ? 1.0f : 0;
		p.load.rear_grip        = z <= 0.0f && y < bbw.y * 0.02f ? 1.0f : 0;
		p.load.left_grip        = x <= 0.0f && y < bbw.y * 0.02f ? 1.0f : 0;
		p.load.right_grip       = 0.0f <  x && y < bbw.y * 0.02f ? 1.0f : 0;
                        
		// アクセル係数
		if( 0 < bbw.y ) { p.load.accel = ( 1.0f - yr ) * mass * 10.0f * scale_factor; }
		p.load.jump = mass * 500.0f;
	}

	sv->update_mass();
}

void update_friction(
	DataProvider*					dp,
	PartixUser::softvolume_type*	sv,
	bool							resting )
{
	float normal_friction = dp->GetFrictionFactor();

	PartixUser::mesh_type*   mesh   = sv->get_mesh();
	PartixUser::points_type& points = mesh->get_points();
	for( PartixUser::points_type::iterator i = points.begin() ;
		 i != points.end() ;
		 ++i ) {
		PartixUser::point_type& p = *i;
		const Vector& v = p.source_position;
		
		if( resting ) {
			p.friction = normal_friction; //* float(1.5);
#if 0
		} else if( 0.375f * 0.7f < v.z ) {
			// 鼻面は低摩擦
			p.friction = 2.0f;
#endif
		} else {
			// 前足は低摩擦
			p.friction = 
				float(0) < v.z && v.y < float(0.02) * float(0.7) ?
				normal_friction / float(2) : normal_friction;
		}
		if(99999.0f < p.friction) {
			DebugBreak();
		}
	}
}

void accel_entity(
	DataProvider*					dp,
	PartixUser::softvolume_type*	sv,
	const Vector&					accel )
{
	float accel_factor = dp->GetAccelFactor();

	PartixUser::mesh_type*   mesh     = sv->get_mesh();
	PartixUser::points_type& vertices = mesh->get_points();
	for( PartixUser::points_type::iterator i = vertices.begin() ;
		 i != vertices.end() ;
		 ++i ) {
		PartixUser::point_type& p = *i;
		p.forces += accel * p.load.accel * accel_factor;
	}
}

void jump_entity(
	DataProvider*					dp,
	PartixUser::softvolume_type*	sv )
{
	//float accel_factor = dp->GetAccelFactor();
	Vector gravity(0, -9.8f, 0);
	Vector up = -gravity; up.Normalize();

	PartixUser::mesh_type*   mesh     = sv->get_mesh();
	PartixUser::points_type& vertices = mesh->get_points();
	for( PartixUser::points_type::iterator i = vertices.begin() ;
		 i != vertices.end() ;
		 ++i ) {
		PartixUser::point_type& p = *i;
		p.forces += up * p.load.jump;
	}
}

} // namespace

FourLegs::FourLegs()
{
	//default_data_provider_ = new DefaultDataProvider;
	default_data_provider_ = new DBDataProvider;
	data_provider_ = default_data_provider_;
    user_accel_ = Vector(0, 0, 0);

	static int id_source = 0;
	id_ = id_source++;
}

FourLegs::~FourLegs()
{
	delete default_data_provider_;	
}

void	_flFASTCALL FourLegs::Move()
{
	DataProvider* dp = data_provider_;

	Physie* physie = GetPhysie();
	if( !physie ) { return; }

	Matrix curr_orientation;
	GetOrientation(&curr_orientation);

	const Matrix& pm = orientation_;
	const Matrix& cm = curr_orientation;

	PartixUser::softvolume_type* sv =
		reinterpret_cast<PartixUser::softvolume_type*>(
			GetPartixModule()->GetPhysieBody(physie));

	const float dt = PARTIX_TICK;

	AnalyzeData ad;
	analyze(
		data_provider_,
		sv,
		dt,
		user_accel_,
		velocity_,
		pm,
		cm,
		ad,
		sensory_balance_ );

#if 0
	if( id_ == 0 ) {
		char buffer[256];
		sprintf_s( buffer, "bb %f, %f, %f: mass %f\n",
				   ad.bbmax.x - ad.bbmin.x,
				   ad.bbmax.y - ad.bbmin.y,
				   ad.bbmax.z - ad.bbmin.z,
				   ad.mass );
		OutputDebugStringA( buffer );
	}
#endif


#if 0
	if( id_ == 0 ) {
		char buffer[256];
		sprintf( buffer, "current velocity %f\n",
				 ad.curr_velocity.Length() );
		OutputDebugStringA( buffer );

	}
#endif

	// 重力
	Vector gravity(0, -9.8f, 0);
	Vector up = -gravity; up.Normalize();

	////////////////////////////////////////////////////////////////
	// 姿勢
	Quaternion q0;
	Vector physical_rotation_axis;
	float	physical_rotation_angle;
	{
		get_physical_rotation(
			orientation_,
			curr_orientation,
			q0,
			physical_rotation_axis,
			physical_rotation_angle );
		check_quaternion(q0,"q0");
		physical_rotation_angle /= dt;
	}

	// バランス
	Quaternion balance_q = q0; // prev->balanced
	float balance_dot = 1.0f;
	if (0 < ad.total_grip) {
		float max_balance_angle =
			flMath::DEGtoRAD(dp->GetBalanceAngleMax()) * dt;
		Vector balance_axis;
		if (get_balance_rotation(
				pm, cm, balance_axis, balance_dot, sensory_balance_)){
			if (ad.left_grip == 0 && 0 < ad.right_grip) {
				// 左足が浮いてるときは右には傾かない
				flMESSAGE( "losing left grip" ); 
				if (ad.front.Dot(&balance_axis) < 0) {
					flMESSAGE( "cancel balance ad.front %f, %f, %f",
							   ad.front.x, ad.front.y, ad.front.z ); 
					flMESSAGE( "cancel balance balance_axis %f, %f, %f",
							   balance_axis.x, balance_axis.y, balance_axis.z ); 
					flMESSAGE( "cancel balance %f", ad.front.Dot(&balance_axis)); 
					balance_dot = float(1);
				}
			}
			if (ad.right_grip == 0 && 0 < ad.left_grip) {
				// 右足が浮いてるときは左には傾かない
				flMESSAGE( "losing right grip" ); 
				if (0 < ad.front.Dot(&balance_axis)) {
					flMESSAGE( "cancel balance ad.front %f, %f, %f",
							   ad.front.x, ad.front.y, ad.front.z ); 
					flMESSAGE( "cancel balance balance_axis %f, %f, %f",
							   balance_axis.x, balance_axis.y, balance_axis.z ); 
					flMESSAGE( "cancel balance %f", ad.front.Dot(&balance_axis)); 
					balance_dot = float(1);
				}
			}

			float balance_angle = acosf(balance_dot);
			//flMESSAGE( "balance angle %f", balance_angle );

			if (max_balance_angle < balance_angle) {
				balance_angle = max_balance_angle;
			}

#if 0
			if( id_ == 0 ) {
				char buffer[256];
				sprintf( buffer, "balance angle %f / %f\n",
						 flMath::RADtoDEG( balance_angle ),
						 flMath::RADtoDEG( max_balance_angle ) );
				OutputDebugStringA( buffer );
			}
#endif

			quat_from_axis_angle(&balance_axis, balance_angle, &balance_q);
			check_quaternion(balance_q,"balance_q");
		}
	}
                
#if 0
	// バンク
	float bank = get_bank(ad.prev_velocity, ad.curr_velocity);
#endif

	//// ハンドル
	Quaternion turning_q = q0;
	float turning_angle = 0;
	if( 0 < ad.total_grip ) { 
		float max_turning_angle =
			flMath::DEGtoRAD(dp->GetTurningAngleMax()) * dt *
			ad.bottom_grip * ad.control_power;
		Vector turning_axis;
		float turning_dot;
		if( float(0.7) < balance_dot &&
			get_turning_rotation(
				pm, cm, ad.control_accel, turning_axis, turning_dot ) ) {
			turning_angle = acosf( turning_dot ) * ad.front_grip;

			//float t = 1.0f + ( 2.0f - turning_dot ) * 0.2f;
			float t = float(0.6) + ( float(2.0) - turning_dot ) * float(0.1);
			max_turning_angle *= t;
			if( max_turning_angle < turning_angle ) {
				turning_angle = max_turning_angle;
			}

#if 0
			if( id_ == 0 ) {
				char buffer[256];
				sprintf( buffer, "turning angle %f / %f\n",
						 flMath::RADtoDEG( turning_angle ),
						 flMath::RADtoDEG( max_turning_angle ) );
				OutputDebugStringA( buffer );
			}
#endif

			quat_from_axis_angle( &turning_axis, turning_angle, &turning_q );
			check_quaternion(turning_q, "turning_q");
		}
	}

	Vector control_force = ad.control_accel;

	// のけぞり制御
	Quaternion backbend_q = q0;
	{
		float backbend_balance = ad.front_grip - ad.rear_grip;
		backbend_balance_ =
			backbend_balance_ +
			( backbend_balance - backbend_balance_ ) * 0.5f;
		if( 0.5f < backbend_balance_ ) { backbend_balance_ = 0.5f; }
                        
		if( 0.0f < backbend_balance_ &&
			-0.01f < get_axisangle(ad.front, ad.old_front, ad.right) ){ 
			float backbend_angle =
				-backbend_balance_ * dp->GetBackbendAngleFactor();

			quat_from_axis_angle(&ad.right, backbend_angle, &backbend_q);
			check_quaternion(backbend_q, "backbend_q");
                                
			// WARNING: control_accel書き換えてる
			control_force *= 1.0f - backbend_angle * 100.0f; 
		}
	}
		
	// 合成
	orientation_ = curr_orientation;

	check_quaternion(q0,"q0");
	check_quaternion(balance_q,"balance");
	check_quaternion(turning_q,"turning");
	check_quaternion(backbend_q,"backbend");

	Quaternion q;
	//blend_quaternion(q, bank_q, dp->GetBankRatio());
	q = slerp(balance_q, dp->GetBalanceRatio());
	q = slerp(q, turning_q, dp->GetTurningRatio());
	q = slerp(q, backbend_q, dp->GetBackbendRatio());
	check_quaternion(q,"normalized");

	Quaternion total_q = q0; total_q.Inverse();
	total_q *= q; // curr -> controlled curr
	total_q.normalize();
	sv->rotate(total_q.w, total_q.x, total_q.y, total_q.z);

	update_friction(dp, sv, user_accel_.Length() < float(0.1));

#if 1
	////////////////////////////////////////////////////////////////
	// 加速
	if (0 < ad.total_grip) {
		Vector control_dir = control_force;
		float control_len = control_dir.Length();
		control_dir.Normalize();

		if( balance_dot < 0.5f && 0.5f < control_len ) {
			// 体が左右に傾きすぎ
			if (ad.left_grip == 0 && control_dir.Dot(&ad.right)< -0.5f) {
				flMESSAGE( "FORCE LEFT\n" );
				Quaternion q; quat_from_axis_angle(&ad.front, float(2.0)*dt, &q);
				check_quaternion(q, "FORCE LEFT");
				sv->rotate(q.w, q.x, q.y, q.z);
			}
			if (ad.right_grip == 0 && 0.5f < control_dir.Dot(&ad.right)) {
				flMESSAGE( "FORCE RIGHT\n" );
				Quaternion q; quat_from_axis_angle(&ad.front, float(-2.0)*dt, &q);
				check_quaternion(q, "FORCE RIGHT");
				sv->rotate(q.w, q.x, q.y, q.z);
			}
		} else if (ad.speed < 1.0f && ad.front_grip < 0.1f) {
			// 前足が接地してない
			float dot = up.Dot(&ad.front);
			if ( 0 < dot ) {
				//OutputDebugStringA("standing\n");
				accel_entity(dp, sv, control_force * 4.0f * dot);
			}
		} else {
			Vector accel(0, 0, 0);
			if (0.0001f < ad.control_power) {
				// 進行ベクトル(前方ベクトルへの射影)
				float accel_grip = ad.bottom_grip;
				accel = ad.front * ad.front.Dot(&control_force) * accel_grip;

				// turbo
				if( ad.speed < dp->GetTurboThreshold() ) {
					float k =
						( dp->GetTurboThreshold() - ad.speed ) /
						dp->GetTurboThreshold();
					
					float m =
						k * ( dp->GetTurboMultiplier() - 1.0f ) + 1.0f;

					accel = accel * m;
				}
#if 0
				flMESSAGE("ad.front %f, %f, %f",
						  ad.front.x, ad.front.y, ad.front.z);
				flMESSAGE("control_force %f, %f, %f",
						  control_force.x, control_force.y, control_force.z);
				flMESSAGE("accel %f, %f, %f", accel.x, accel.y, accel.z);
#endif
				accel_entity(dp, sv, accel);
			}
		}
	}
#endif
}

void	_flFASTCALL FourLegs::SetAccel(const Vector& av)
{
	Vector v = av;

	// 急旋回はブレーキ
	{
		//Vector v0 = m_cm(2); v0.normalize();
		Vector v0 = velocity_; v0.Normalize();
		Vector v1 = v;			v1.Normalize();
		if (flMath::DEGtoRAD(data_provider_->GetBrakeAngle()) <
			fabsf(acosf(v0.Dot(&v1))) &&
			3.0f < velocity_.Length()) {
			flMESSAGE("BREAKING: %f\n", acosf(v0.Dot(&v1)));
			v = Vector(0, 0, 0);
		}
	}

	float len0 = user_accel_.Length();
	float len1 = v.Length();

	// 急激な変化防止
	// スピードが速いほど旋回しにくい
	float speed = velocity_.Length();
	float vr = PARTIX_TICK * 5.0f;
	float tr = PARTIX_TICK * 20.0f;

	// 進行方向を球面補完
	bool f = false;
	if (float(0.01)< len0 && float(0)< len1) {
		Vector v0 = user_accel_; v0.Normalize();
		Vector v1 = v;           v1.Normalize();
		float dot = v0.Dot(&v1);
		if (fabsf(dot)< 1.0f-1.0e-6f) {
			Vector axis; axis.Cross(&v0, &v1);
			if(1.0f-1.0e-6f <= axis.Length()) {
				axis.Normalize();
				float angle = acosf(dot);
				Quaternion q; quat_from_axis_angle(&axis, angle * tr, &q);
				check_quaternion(q, "direction0");
				Quaternion rq = q; rq.Inverse();
				check_quaternion(rq, "direction1");
				Quaternion t(user_accel_.x, user_accel_.y, user_accel_.z, 1.0f);
				check_quaternion(t, "direction2");
				Quaternion tt(q);
				tt.Mul(&t);
				tt.Mul(&rq);

				user_accel_ = Vector(tt.x, tt.y, tt.z);
                check_vector(user_accel_);
				float newlen = len0 * (1.0f - vr) + len1 * vr;
				user_accel_ *= newlen / len0;
                check_vector(user_accel_);
				f = true;
			}
		}
	}

	if( !f ) {
		// TODO: あまり正確でない
		user_accel_ = user_accel_ * ( 1.0f - vr ) + v * vr;
        check_vector(user_accel_);
	}
}

void	_flFASTCALL FourLegs::DoJump()
{
	DataProvider* dp = data_provider_;

	PartixUser::softvolume_type* sv =
		reinterpret_cast<PartixUser::softvolume_type*>(
			GetPartixModule()->GetPhysieBody(GetPhysie()));

	jump_entity(dp, sv);
}

void	_flFASTCALL FourLegs::AddExternalForce(const Vector& v)
{
	PartixUser::softvolume_type* sv =
		reinterpret_cast<PartixUser::softvolume_type*>(
			GetPartixModule()->GetPhysieBody(GetPhysie()));
	sv->set_force(v);
}

void	_flFASTCALL FourLegs::OnAssignPhysie(
	Physie* physie, float scale, float mass)
{
	PartixUser::softvolume_type* sv =
		reinterpret_cast<PartixUser::softvolume_type*>(
			GetPartixModule()->GetPhysieBody(physie));

	setup_physie(
		sv,
		0,
		float(-2.0),
		float(2.0),
		mass,
		scale
		);

	sensory_balance_ = 0;
	backbend_balance_ = 0;
	user_accel_ = Vector(0, 0, 0);
	velocity_ = Vector(0, 0, 0);
}
