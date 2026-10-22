// Copyright 2017 Intel Corporation. Apache-2.0; see LICENSE.
// Verbatim reference fragments, not a standalone build target.
// Source commit: 75d5579aa258d97157d8a29b1c4da32f3d0a7608

// include/misc_image.h:48-50
	float3   deprojectz(const float2 &p, float d)       const { return float3((p.x - principal().x) / focal().x, (p.y - principal().y) / focal().y, 1.0f) * d; }
	float3   deprojectz(const int2 &p,unsigned short d) const { return deprojectz(asfloat2(p), (float)d); }
	float2   projectz(const float3 &v)                  const { return v.xy() / v.z * focal() + principal(); }

// include/misc_image.h:409-417
template<class T> std::vector<float3> PointCloud(const Image<T> &dimage, float2 filter_range)
{
	std::vector<float3> pointcloud;
	float d;
	for (int2 p : rect_iteration(dimage.dim()))  // p.y and p.x iterate over dimensions of image
		if (FallsWithinRange(d = (dimage.pixel(p)* dimage.cam.depth_scale), filter_range))
			pointcloud.push_back(dimage.cam.deprojectz(float2(p), d));
	return pointcloud;
}

// include/physmodel.h:163-180
static bool g_cloud_directed=1;
inline LimitLinear CloudConstraint(const std::vector<RigidBody*> &rigidbodies, float3 v, float3 origin = { 0,0,0 })
{
	int k = 0;
	RigidBody *rb; float4 p;
	std::tie(rb, p) = closest(rigidbodies, v);
	::HitInfo h;
	if (g_cloud_directed&&dot(v-origin, p.xyz()) > 0 && (h = ConvexHitCheck(rb->shapes[0].planes, rb->pose(), origin, v)))    //  in case we are closer to backside and point is behind shape
		return  ConstrainAlongDirection(NULL, v, rb, rb->pose().inverse()*h.impact, normalize(v - origin), -1.0f, 1.0f);  // dont know where on surface so use plane through centroid - no extra pushback 
																														  // note could try impulse range [0.0..1.0] so only pushing (to be tested)
	return ConstrainAlongDirection(NULL, v, rb, rb->pose().inverse()*(v - p.xyz()*dot(p, float4(v, 1))), p.xyz(), -1.0f, 1.0f);
}
inline std::vector<LimitLinear> CloudConstraints(const std::vector<RigidBody*> &rigidbodies, const std::vector<float3> &points, float3 origin = { 0,0,0 })
{
	std::vector<LimitLinear> linears;
	for (auto v : points)
		linears.push_back(CloudConstraint(rigidbodies, v, origin));
	return linears;

// include/physmodel.h:345-356
	void FitPointCloud(const std::vector<float3> &points, std::vector<LimitLinear> linears = {}, std::vector<LimitAngular> angulars = {},float microforce=1.0f)  // simplistic fit
	{
		auto setmaxforce_u = [this,microforce](std::vector<LimitLinear>&& linears) ->std::vector<LimitLinear> { for (auto &c : linears) c.forcelimit = float2(-1.0f,1.0f) *( (c.rb1 == &this->rigidbodies[0] || c.rb1 == &this->rigidbodies[1] || c.rb1 == &this->rigidbodies[2]) ? physics_weak_force: 1.0f)*microforce; return linears; };
		Append(linears, setmaxforce_u( CloudConstraints(Addresses(rigidbodies), points )));
		auto setmaxforce_s = [](LimitLinear c) ->LimitLinear { c.forcelimit = float2(-10.0f, 10.0f); return c; };
		Append(linears , GetLinearConstraints ());
		Append(angulars, GetAngularConstraints());
		// angulars.push_back(ConstrainConeAngle(NULL, float3(0, -1, 0), rigidbodies[0], { 0, 0, 1 }, 70.0f));  // orient object upward

		PhysicsUpdate(Addresses(rigidbodies), linears, angulars, {});
		SanityCheck();
	}
