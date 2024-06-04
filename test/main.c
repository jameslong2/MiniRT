#include "mlx.h"
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <assert.h>
#define PI 22/7
#define EPSILON 0.000001
typedef struct {
    float x, y, z;
} Vector3;

typedef struct {
    float x, y;
} Vector2;

typedef struct {
    float x, y, z, w;
} Quaternion;

typedef struct {
    Vector3 position;
    Quaternion rotation;
    float near_plane;
    float far_plane;
    float fov;
} Camera;

typedef struct {
    int *triangles;
    Vector3 *vertices;
    Vector2 *uvs;
    int vert_count;
    int tri_count;
    int uv_count;
    Vector3 color;
} Object;

typedef struct {
    Vector3 position;
    float radius;
    float intensity;
} PointLight;

typedef struct {
    void *mlx;
    void *win;
    void *img;
    int		bits_per_pixel;
	int		line_length;
	int		endian;
    char *addr;
    int width;
    int height;
    int *framebuffer;
} Renderer;
// Utility functions for vector operations
Vector3 vector_add(Vector3 a, Vector3 b) {
    return (Vector3){a.x + b.x, a.y + b.y, a.z + b.z};
}

Vector3 vector_sub(Vector3 a, Vector3 b) {
    return (Vector3){a.x - b.x, a.y - b.y, a.z - b.z};
}

Vector3 vector_scale(Vector3 v, float scale) {
    return (Vector3){v.x * scale, v.y * scale, v.z * scale};
}

float vector_dot(Vector3 a, Vector3 b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}
Vector3 vector_mul(Vector3 a, Vector3 b) {
    return (Vector3){a.x * b.x , a.y * b.y , a.z * b.z};
}
Vector3 vector_cross(Vector3 a, Vector3 b) {
    return (Vector3){
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}

Vector3 vector3_scale_add(Vector3 a, float scale, Vector3 b) {
    Vector3 scaled_a = vector_scale(a, scale);
    return vector_add(scaled_a, b);
}

Vector3 vector_normalize(Vector3 v) {
    float length = sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
    return (Vector3){v.x / length, v.y / length, v.z / length};
}

typedef struct {
    Vector3 origin;
    Vector3 direction;
} Ray;

typedef struct {
    int hit;
    float distance;
    Vector3 point;
    Vector3 normal;
    Vector2 uv;
    float tnear;
    Vector2 coords;
} Intersection;
float vector_length(Vector3 v) {
    return sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}
float vector_area(Vector3 a, Vector3 b, Vector3 c) {
    Vector3 ab = vector_sub(b, a);
    Vector3 ac = vector_sub(c, a);
    return vector_length(vector_cross(ab, ac)) * 0.5f;
}
float clamp(float value, float min, float max) {
    return fmax(min, fmin(value, max));
}

Ray create_primary_ray(const Camera *camera, int x, int y, int width, int height);
Vector2 get_texture_coordinates(const Vector3 hit_point, const Vector3 v0, const Vector3 v1, const Vector3 v2, const Vector2 uv0, const Vector2 uv1, const Vector2 uv2);
///////
Vector2 get_texture_coordinates(const Vector3 hit_point, const Vector3 v0, const Vector3 v1, const Vector3 v2, const Vector2 uv0, const Vector2 uv1, const Vector2 uv2) {
    // Compute the areas and factors
    float area_total = vector_area(v0, v1, v2);
    float area0 = vector_area(hit_point, v1, v2) / area_total;
    float area1 = vector_area(hit_point, v2, v0) / area_total;
    float area2 = vector_area(hit_point, v0, v1) / area_total;

    // Interpolate the UV coordinates using the barycentric coordinates
    Vector2 uv;
    uv.x = uv0.x * area0 + uv1.x * area1 + uv2.x * area2;
    uv.y = uv0.y * area0 + uv1.y * area1 + uv2.y * area2;
    
    return uv;
}
// calculo interseccion
Intersection rayIntersectsTriangle(Vector3 og, Vector3 dir, Vector3 vert[]){
    Intersection result;
    Vector3 vert0=vert[0];
    Vector3 vert1=vert[1];
    Vector3 vert2=vert[2];
    Vector3 edge1 = vector_sub(vert1, vert0);
    Vector3 edge2 = vector_sub(vert2, vert0);
    Vector3 h = vector_cross(dir, edge2);
    float a = vector_dot(edge1, h);
    if (a > -EPSILON && a < EPSILON){
        result.hit = 0;
        return (result);
    }
    float f = 1.0 / a;
    Vector3 s = vector_sub(og, vert0);
    float u = f * vector_dot(s, h);
    if (u < 0.0 || u > 1.0){
        result.hit = 0;
        return (result);
    }
    Vector3 q = vector_cross(s, edge1);
    float v = f * vector_dot(dir, q);
    if (v < 0.0 || u + v > 1.0) {
        result.hit = 0;
        return (result);
    }
    float t = f * vector_dot(edge2, q);
    if (t > EPSILON){
        Vector3 point = vector3_scale_add(dir, t, og);
        result.hit = 1;
        result.point = point;
        result.distance = t;
        Vector3 normal = vector_add(vector_add(vector_scale(vert0, 1 - u - v), vector_scale(vert1, u)), vector_scale(vert2, v));
        //
        //Vector3 normal = vector_cross(edge1, edge2);
        //
        result.normal = vector_normalize(normal);
        result.uv = (Vector2){u, v};
        result.tnear = t;
        return (result);
    }else{
        result.hit = 0;
        return (result);
    }
}
//
Intersection intersect_ray_object(Ray ray, Object *object) {
    // Intersection logic (e.g., Möller–Trumbore algorithm) should be implemented here
    int triangle_count = object->tri_count;
    Vector3 vertex[3];
    Intersection closest_intersection = {0};
    float closest_distance = INFINITY;
    for (int i = 0; i < triangle_count/3; i++) {
        
        int idx1 = object->triangles[i * 3];
        int idx2 = object->triangles[i * 3 + 1];
        int idx3 = object->triangles[i * 3 + 2];
        vertex[0] = object->vertices[idx1];
        vertex[1] = object->vertices[idx2];
        vertex[2] = object->vertices[idx3];
        Intersection inter = rayIntersectsTriangle(ray.origin,vector_normalize(ray.direction),vertex);
        if (inter.hit == 1 && inter.distance < closest_distance)
        {
            closest_intersection = inter;
            closest_distance = inter.distance;
        }
    }
    return closest_intersection;
}
float vector_magnitude(Vector3 v) {
    return sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}
float vector3_distance(Vector3 a, Vector3 b) {
    float dx = a.x - b.x;
    float dy = a.y - b.y;
    float dz = a.z - b.z;
    return sqrt(dx * dx + dy * dy + dz * dz);
}


//
Vector3 illuminate(Ray ray, Intersection intersection, PointLight light, Object obj) {
    Vector3 hitPoint = intersection.point; 
    Vector3 hitNormal = intersection.normal; 
    Vector3 lightDir = vector_sub(light.position,hitPoint);
    float albedo = 0.5;
    float r2 = vector3_distance(light.position, hitPoint);
    if (r2 <= 0 || r2 > light.radius) {
        return (Vector3){0,0,0};
    }
    Vector3 lightInt = (Vector3){0.01,0.01,0.01};
    float atenuation = 1 - (r2 / light.radius);
    if (atenuation < 0) {
        atenuation = 0;
    }
    lightInt = vector_scale(lightInt, light.intensity * pow(atenuation,2));
    float val = 0.5/M_PI;
    lightInt.x *= val,lightInt.y *= val,lightInt.z *= val;
    lightInt.x = clamp(lightInt.x, 0.0, 1.0);
    lightInt.y = clamp(lightInt.y, 0.0, 1.0);
    lightInt.z = clamp(lightInt.z, 0.0, 1.0);
    //lightInt.x /= 255,lightInt.y /= 255,lightInt.z /= 255;
    return (lightInt);
}

float randomNumber(){
   return ((float)rand()/RAND_MAX);
}

Vector3 random_unit_vector() {
    float theta = (float)rand() / RAND_MAX * 2.0f * M_PI;
    float phi = (float)rand() / RAND_MAX * M_PI;
    float sin_phi = sinf(phi);

    float x = sin_phi * cosf(theta);
    float y = sin_phi * sinf(theta);
    float z = cosf(phi);

    return (Vector3){x,y,z};
}
float random_float(){
    return 2*((float)rand()/RAND_MAX)-1;
}
// Function to generate a random vector within a hemisphere defined by the normal vector
Vector3 random_hemisphere_direction(Vector3 normal) {
    Vector3 random_dir;
    do{
        random_dir = (Vector3){random_float(),random_float(),random_float()};
    }while (vector_dot(random_dir, normal) < 0.0f);
    return vector_normalize(random_dir);
}

Quaternion angleAxisToQuaternion(float angle, Vector3 axis) {
    Quaternion q;
    float halfAngle = angle * 0.5f;
    float sinHalfAngle = sin(halfAngle);

    q.w = cos(halfAngle);
    q.x = axis.x * sinHalfAngle;
    q.y = axis.y * sinHalfAngle;
    q.z = axis.z * sinHalfAngle;

    return q;
}

Vector3 quaternionMultiplyVector3(Quaternion q, Vector3 v) {
    Vector3 qv = (Vector3){q.x, q.y, q.z};
    Vector3 uv, uuv;
    uv.x = qv.y * v.z - qv.z * v.y;
    uv.y = qv.z * v.x - qv.x * v.z;
    uv.z = qv.x * v.y - qv.y * v.x;
    uuv.x = qv.y * uv.z - qv.z * uv.y;
    uuv.y = qv.z * uv.x - qv.x * uv.z;
    uuv.z = qv.x * uv.y - qv.y * uv.x;
    uv.x *= 2.0f * q.w;
    uv.y *= 2.0f * q.w;
    uv.z *= 2.0f * q.w;
    uuv.x *= 2.0f;
    uuv.y *= 2.0f;
    uuv.z *= 2.0f;
    Vector3 result;
    result.x = v.x + uv.x + uuv.x;
    result.y = v.y + uv.y + uuv.y;
    result.z = v.z + uv.z + uuv.z;

    return result;
}

float vector_angle(Vector3 from, Vector3 to)
{
    return acos(clamp(vector_dot(vector_normalize(from), vector_normalize(to)), -1.f, 1.f)) * 57.29578f;
}

Vector3 uniformSampleHemisphere(float r1, float r2)
{
    // cos(theta) = r1 = y
    // cos^2(theta) + sin^2(theta) = 1 -> sin(theta) = srtf(1 - cos^2(theta))
    float sinTheta = sqrtf(1 - r1 * r1);
    float phi = 2 * M_PI * r2;
    float x = sinTheta * cosf(phi);
    float z = sinTheta * sinf(phi);
    return (Vector3){x, r1, z};
}
Vector3 vector_divide(Vector3 v, float divisor) {
    return (Vector3){v.x / divisor, v.y / divisor, v.z / divisor};
}
void createCoordinateSystem(Vector3 N, Vector3 *Nt, Vector3 *Nb) 
{ 
    if (fabs(N.x) > fabs(N.y))
        *Nt = vector_divide((Vector3){N.z, 0, -N.x}, sqrtf(N.x * N.x + N.z * N.z));
    else
        *Nt = vector_divide((Vector3){0, -N.z, N.y}, sqrtf(N.y * N.y + N.z * N.z));
    *Nb = vector_cross(N, *Nt); 
}

float computeIrradiance(float x, float y, float z) {
    return 1.0f / (1.0f + x * x + y * y + z * z);
}

Vector3 vector_reflect(Vector3 incidentVec, Vector3 normal)
{
    Vector3 out = vector_scale(normal, 2 * vector_dot(incidentVec, normal));
    return vector_sub(incidentVec,out);
}

Vector3 vector_refract(Vector3 incidentVec, Vector3 normal, float eta)
{
    Vector3 out;
    float N_dot_I = vector_dot(normal, incidentVec);
    float k = 1.f - eta * eta * (1.f - N_dot_I * N_dot_I);
    if (k < 0.f)
        out = (Vector3){0.f, 0.f, 0.f};
    else
        out = vector_sub(vector_scale(incidentVec, eta), vector_scale(normal, eta * N_dot_I + sqrtf(k)));
    return (out);
}

Vector3 render(Ray ray, Object *objects, int object_count, PointLight *lights, int light_count, Camera cam) {
    Intersection closest_intersection = {0};
    float closest_distance = INFINITY;
    Object *touch;
    Vector3 color;
    int index = -1;
    Vector3 final_light = (Vector3){0,0,0};
    for (int i = 0; i < object_count; i++) {
        Intersection intersection = intersect_ray_object(ray, &objects[i]);
        if (intersection.hit && intersection.distance < closest_distance) {
            closest_distance = intersection.distance;
            closest_intersection = intersection;
            color = objects[i].color;
            touch = &objects[i];
            index = i;
        }
    }

    if (!closest_intersection.hit) {
        return (Vector3){0.0f, 0.0f, 1.0f}; // Background color
    }

    Vector3 intensity = {0.1,0.1,0.1}; // Ambient intensity
    int i;
    int light_c = 0;
    for (i = 0; i < light_count; i++) {
        Vector3 light_dir = vector_sub(lights[i].position, closest_intersection.point);
        light_dir = vector_normalize(light_dir);
        Ray shadow_ray = {vector_add(closest_intersection.point, vector_scale(closest_intersection.normal, 0.001f)), light_dir};

        int in_shadow = 0;
        for (int j = 0; j < object_count; j++) {
            if (touch != &objects[j]) {
                Intersection shadow_intersection = intersect_ray_object(shadow_ray, &objects[j]);
                if (shadow_intersection.hit && shadow_intersection.distance < vector3_distance(lights[i].position, closest_intersection.point)) {
                    in_shadow = 1;
                    break;
                }
            }
        }

        if (!in_shadow) {
            Vector3 ilu = illuminate(ray, closest_intersection, lights[i], *touch);
            light_dir = vector_sub(closest_intersection.point, lights[i].position);
            light_dir = vector_normalize(light_dir);    
            intensity = vector_add(intensity, vector_scale(ilu, cos(vector_dot(light_dir,closest_intersection.normal))));
            intensity.x = clamp(intensity.x, 0.0, 1.0);
            intensity.y = clamp(intensity.y, 0.0, 1.0);
            intensity.z = clamp(intensity.z, 0.0, 1.0);
        }
        //implementar otras opciones de luz

    }
            //indirect
        Vector3 ind_color = (Vector3){0,0,0};
        Vector3 hitNormal = vector_normalize(closest_intersection.normal);
        for (int x = 0; x < 100 ; x++){
            Vector3 rand_dir = random_hemisphere_direction(hitNormal);
            Ray rand_ray = (Ray){vector_add(closest_intersection.point, vector_scale(hitNormal, 0.001f)), rand_dir};
            float rand_dist = INFINITY;
            Vector3 obj_col = (Vector3){0,0,0};
            for (int z = 0; z < object_count; z++) {
                Intersection rand_int = intersect_ray_object(rand_ray, &objects[z]);
                if (rand_int.hit && rand_int.distance < rand_dist && touch != &objects[z]) {
                    rand_dist = rand_int.distance;
                    obj_col = objects[z].color;
                }
            }
            ind_color = vector_add(ind_color, vector_scale(obj_col, 1));
        }
        ind_color.x /= 100;
        ind_color.y /= 100;
        ind_color.z /= 100;
        intensity = vector_add(intensity, vector_mul(ind_color, intensity));
        //
    intensity = vector_mul(intensity, color);
    intensity.x = clamp(intensity.x, 0.0, 1.0);
    intensity.y = clamp(intensity.y, 0.0, 1.0);
    intensity.z = clamp(intensity.z, 0.0, 1.0);

    if (index == 1)//reflejo para el objeto indice 1, el plano
    {
        Vector3 reflect_color = (Vector3){0,0,0};
        Vector3 R = vector_reflect(ray.direction, (Vector3){0,1,0});
        Ray refl_ray = {vector_add(closest_intersection.point, vector_scale(vector_normalize(R), 0.001f)), R};
        reflect_color = render(refl_ray, objects, object_count, lights, light_count, cam);
        intensity = vector_add(intensity, vector_scale(reflect_color,0.5));//0.5 el valor de reflejo
        intensity.x = clamp(intensity.x, 0.0, 1.0);
        intensity.y = clamp(intensity.y, 0.0, 1.0);
        intensity.z = clamp(intensity.z, 0.0, 1.0);
    }
    /*if (index == 3)//calculo de la refraccion para el objecto indice 3, esfera3
    {
        Vector3 refract_color = (Vector3){0,0,0};
        Vector3 R = vector_refract(ray.direction, closest_intersection.normal, 1.02);
        Ray refract_ray = {vector_add(closest_intersection.point, vector_scale(vector_normalize(R), 0.001f)), R};
        refract_color = render(refract_ray, objects, object_count, lights, light_count, cam);
        intensity = vector_scale(refract_color,0.9);//poner aqui la transparencia del objeto
        intensity.x = clamp(intensity.x, 0.0, 1.0);
        intensity.y = clamp(intensity.y, 0.0, 1.0);
        intensity.z = clamp(intensity.z, 0.0, 1.0);
    }*/
    //final_light = vector_add(final_light, vector_scale(color, intensity));
    return intensity; // Placeholder color
}

void	my_mlx_pixel_put(Renderer *data, int x, int y, int color)
{
	char	*dst;

	dst = data->addr + (y * data->line_length + x * (data->bits_per_pixel / 8));
	*(unsigned int*)dst = color;
}
int fibo(int num, int wi){
    int nextTerm = 1;
    int t1=0,t2=1;
    for (int i = 3; i <= wi; ++i) {
    t1 = t2;
    t2 = nextTerm;
    nextTerm = t1 + t2;
    if (num == nextTerm)
    {
        return 1;
    }
    }
    return 0;
}
void render_scene(Renderer *renderer, Camera *camera, Object *objects, int object_count, PointLight *lights, int light_count) {
    for (int y = 0; y < renderer->height; y++) {
        for (int x = 0; x < renderer->width; x++) {
            Ray primary_ray = create_primary_ray(camera, x, y, renderer->width, renderer->height);
            Vector3 color = render(primary_ray, objects, object_count, lights, light_count, *camera);
            my_mlx_pixel_put(renderer, x, y, (int)(color.x * 255) << 16 | (int)(color.y * 255) << 8 | (int)(color.z * 255));
            //my_mlx_pixel_put(renderer, x, y, (int)(fibo(x,x)) << 16 | (int)(fibo(x,x)) << 8 | (int)(fibo(x,x)));
        }
    }
}

void quaternion_to_matrix(Quaternion q, float matrix[3][3]) {
    matrix[0][0] = 1 - 2 * q.y * q.y - 2 * q.z * q.z;
    matrix[0][1] = 2 * q.x * q.y - 2 * q.z * q.w;
    matrix[0][2] = 2 * q.x * q.z + 2 * q.y * q.w;

    matrix[1][0] = 2 * q.x * q.y + 2 * q.z * q.w;
    matrix[1][1] = 1 - 2 * q.x * q.x - 2 * q.z * q.z;
    matrix[1][2] = 2 * q.y * q.z - 2 * q.x * q.w;

    matrix[2][0] = 2 * q.x * q.z - 2 * q.y * q.w;
    matrix[2][1] = 2 * q.y * q.z + 2 * q.x * q.w;
    matrix[2][2] = 1 - 2 * q.x * q.x - 2 * q.y * q.y;
}

// Transform a vector by a rotation matrix
Vector3 transform_vector(Vector3 v, float matrix[3][3]) {
    return (Vector3){
        v.x * matrix[0][0] + v.y * matrix[0][1] + v.z * matrix[0][2],
        v.x * matrix[1][0] + v.y * matrix[1][1] + v.z * matrix[1][2],
        v.x * matrix[2][0] + v.y * matrix[2][1] + v.z * matrix[2][2]
    };
}

void quaternion_to_rotation_matrix(const Quaternion *q, float matrix[4][4]) {
    float xx = q->x * q->x;
    float yy = q->y * q->y;
    float zz = q->z * q->z;
    float xy = q->x * q->y;
    float xz = q->x * q->z;
    float yz = q->y * q->z;
    float wx = q->w * q->x;
    float wy = q->w * q->y;
    float wz = q->w * q->z;

    matrix[0][0] = 1.0f - 2.0f * (yy + zz);
    matrix[0][1] = 2.0f * (xy - wz);
    matrix[0][2] = 2.0f * (xz + wy);
    matrix[0][3] = 0.0f;

    matrix[1][0] = 2.0f * (xy + wz);
    matrix[1][1] = 1.0f - 2.0f * (xx + zz);
    matrix[1][2] = 2.0f * (yz - wx);
    matrix[1][3] = 0.0f;

    matrix[2][0] = 2.0f * (xz - wy);
    matrix[2][1] = 2.0f * (yz + wx);
    matrix[2][2] = 1.0f - 2.0f * (xx + yy);
    matrix[2][3] = 0.0f;

    matrix[3][0] = 0.0f;
    matrix[3][1] = 0.0f;
    matrix[3][2] = 0.0f;
    matrix[3][3] = 1.0f;
}

Ray create_primary_ray(const Camera *camera, int x, int y, int width, int height) {
    Ray ray;
    ray.origin = camera->position;

    float aspect_ratio = (float)width / height;
    float fov_rad = camera->fov * (PI / 180.0f);
    float scale = tan(fov_rad * 0.5f) * camera->near_plane;

    float pixel_ndc_x = (x + 0.5f) / (float)width;
    float pixel_ndc_y = (y + 0.5f) / (float)height;

    float pixel_screen_x = (2.0f * pixel_ndc_x - 1.0f) * aspect_ratio * scale;
    float pixel_screen_y = (1.0f - 2.0f * pixel_ndc_y) * scale;

    Vector3 pixel_camera_space = {pixel_screen_x, pixel_screen_y, -camera->near_plane};

    float rotation_matrix[4][4];
    quaternion_to_rotation_matrix(&camera->rotation, rotation_matrix);

    Vector3 ray_direction = {
        rotation_matrix[0][0] * pixel_camera_space.x + rotation_matrix[0][1] * pixel_camera_space.y + rotation_matrix[0][2] * pixel_camera_space.z,
        rotation_matrix[1][0] * pixel_camera_space.x + rotation_matrix[1][1] * pixel_camera_space.y + rotation_matrix[1][2] * pixel_camera_space.z,
        rotation_matrix[2][0] * pixel_camera_space.x + rotation_matrix[2][1] * pixel_camera_space.y + rotation_matrix[2][2] * pixel_camera_space.z
    };

    ray.direction = vector_normalize(ray_direction);
    return ray;
}

void allocate_memory_for_sphere(Object *sphere, int segment_count) {
    int vertex_count = (segment_count + 1) * (segment_count + 1);
    int triangle_count = segment_count * segment_count * 2;

    sphere->vertices = (Vector3*)malloc(sizeof(Vector3) * vertex_count);
    sphere->triangles = (int*)malloc(sizeof(int) * triangle_count * 3);
    // UVs are not used in this example
}

void free_memory_for_sphere(Object *sphere) {
    free(sphere->vertices);
    free(sphere->triangles);
    // Free UVs if allocated
}

void create_sphere_mesh(Object *sphere, float radius, int segment_count) {
    allocate_memory_for_sphere(sphere, segment_count);

    int vertex_count = 0;
    int triangle_count = 0;

    for (int i = 0; i <= segment_count; ++i) {
        for (int j = 0; j <= segment_count; ++j) {
            float theta = (float)i / segment_count * PI;
            float phi = (float)j / segment_count * 2.0f * PI;

            float x = radius * sinf(theta) * cosf(phi);
            float y = radius * cosf(theta);
            float z = radius * sinf(theta) * sinf(phi);

            sphere->vertices[vertex_count++] = (Vector3){x, y, z};
        }
    }

    for (int i = 0; i < segment_count; ++i) {
        for (int j = 0; j < segment_count; ++j) {
            int first = (i * (segment_count + 1)) + j;
            int second = first + segment_count + 1;

            sphere->triangles[triangle_count++] = first;
            sphere->triangles[triangle_count++] = second;
            sphere->triangles[triangle_count++] = first + 1;

            sphere->triangles[triangle_count++] = second;
            sphere->triangles[triangle_count++] = second + 1;
            sphere->triangles[triangle_count++] = first + 1;
        }
    }
    sphere->vert_count = vertex_count;
    sphere->tri_count = triangle_count;
}

void translate_object(Object *object, Vector3 translation) {
    for (int i = 0; i < object->vert_count; ++i) {
        object->vertices[i].x += translation.x;
        object->vertices[i].y += translation.y;
        object->vertices[i].z += translation.z;
    }
}

void create_plane_mesh(Object *plane, float width, float height, int resolution_x, int resolution_z);
void free_plane_mesh(Object *plane);

void create_plane_mesh(Object *plane, float width, float height, int resolution_x, int resolution_z) {
    int vertex_count = (resolution_x + 1) * (resolution_z + 1);
    int triangle_count = resolution_x * resolution_z * 2;
    plane->tri_count = triangle_count;
    plane->vert_count = vertex_count;
    plane->vertices = (Vector3*)malloc(sizeof(Vector3) * vertex_count);
    plane->triangles = (int*)malloc(sizeof(int) * triangle_count * 3);

    float half_width = width / 2.0f;
    float half_height = height / 2.0f;
    float dx = width / resolution_x;
    float dz = height / resolution_z;

    int vertex_index = 0;
    for (int z = 0; z <= resolution_z; ++z) {
        for (int x = 0; x <= resolution_x; ++x) {
            plane->vertices[vertex_index++] = (Vector3){
                x * dx - half_width,
                0,
                z * dz - half_height
            };
        }
    }

    int triangle_index = 0;
    for (int z = 0; z < resolution_z; ++z) {
        for (int x = 0; x < resolution_x; ++x) {
            int top_left = z * (resolution_x + 1) + x;
            int top_right = top_left + 1;
            int bottom_left = top_left + (resolution_x + 1);
            int bottom_right = bottom_left + 1;

            // First triangle
            plane->triangles[triangle_index++] = top_left;
            plane->triangles[triangle_index++] = bottom_left;
            plane->triangles[triangle_index++] = top_right;

            // Second triangle
            plane->triangles[triangle_index++] = top_right;
            plane->triangles[triangle_index++] = bottom_left;
            plane->triangles[triangle_index++] = bottom_right;
        }
    }
}

void generate_cylinder_mesh(Object *obj, float radius, float height, int segments) {
    int vertex_count = (segments + 1) * 2 + 2;
    int triangle_count = segments * 4;

    Vector3 *vertices = (Vector3 *)malloc(vertex_count * sizeof(Vector3));
    int *triangles = (int *)malloc(triangle_count * 3 * sizeof(int));

    // Create vertices
    float half_height = height / 2.0f;

    // Bottom center vertex
    vertices[0] = (Vector3){0.0f, -half_height, 0.0f};
    // Top center vertex
    vertices[1] = (Vector3){0.0f, half_height, 0.0f};

    // Bottom circle vertices
    for (int i = 0; i <= segments; ++i) {
        float theta = (float)i / segments * 2.0f * M_PI;
        float x = radius * cos(theta);
        float z = radius * sin(theta);
        vertices[i + 2] = (Vector3){x, -half_height, z};
    }

    // Top circle vertices
    for (int i = 0; i <= segments; ++i) {
        float theta = (float)i / segments * 2.0f * M_PI;
        float x = radius * cos(theta);
        float z = radius * sin(theta);
        vertices[i + segments + 3] = (Vector3){x, half_height, z};
    }

    // Create triangles
    int tri_index = 0;

    // Bottom circle triangles
    for (int i = 0; i < segments; ++i) {
        triangles[tri_index++] = 0;
        triangles[tri_index++] = i + 2;
        triangles[tri_index++] = i + 3;
    }

    // Top circle triangles
    for (int i = 0; i < segments; ++i) {
        triangles[tri_index++] = 1;
        triangles[tri_index++] = i + segments + 3;
        triangles[tri_index++] = i + segments + 4;
    }

    // Side triangles
    for (int i = 0; i < segments; ++i) {
        int bottom1 = i + 2;
        int bottom2 = i + 3;
        int top1 = i + segments + 3;
        int top2 = i + segments + 4;

        // First triangle
        triangles[tri_index++] = bottom1;
        triangles[tri_index++] = bottom2;
        triangles[tri_index++] = top1;

        // Second triangle
        triangles[tri_index++] = bottom2;
        triangles[tri_index++] = top2;
        triangles[tri_index++] = top1;
    }

    *obj = (Object){triangles,vertices,0 vertex_count, triangle_count,0,(Vector3){0,0,0}};
}

int main() {
    srand(time(0));
    Renderer renderer;
    renderer.width = 1600;
    renderer.height = 1200;
    renderer.mlx = mlx_init();
    renderer.win = mlx_new_window(renderer.mlx, renderer.width, renderer.height, "Ray Tracer");
    renderer.img = mlx_new_image(renderer.mlx, 1600, 1200);
    renderer.addr = mlx_get_data_addr(renderer.img, &renderer.bits_per_pixel, &renderer.line_length,
								&renderer.endian);
    Camera camera = {{0, 3, 7}, {-0.1305262, 0, 0, 0.9914449}, 0.001f, 100.0f, 60.0f};
    Object sphere;
    create_sphere_mesh(&sphere, 1, 12);
    sphere.color = (Vector3){1,1,1};
    Object sphere2;
    Object sphere3;
    Object sphere4;
    create_sphere_mesh(&sphere2, 1, 12);
    create_sphere_mesh(&sphere3, 1, 12);
    create_sphere_mesh(&sphere4, 1, 12);
    sphere2.color = (Vector3){1,0,0};
    sphere3.color = (Vector3){0,1,0};
    sphere4.color = (Vector3){0,0,1};
    translate_object(&sphere, (Vector3){0,0,0});
    translate_object(&sphere2, (Vector3){2,0,0});
    translate_object(&sphere3, (Vector3){-2,0,0});
    translate_object(&sphere4, (Vector3){0,2,0});
    Object plane;
    plane.color = (Vector3){1,1,1};
    create_plane_mesh(&plane, 20, 20, 2, 2);
    translate_object(&plane, (Vector3){5,-1,5});
    Object objects[] = {sphere, plane, sphere2,sphere3,sphere4};
    int object_count = sizeof(objects) / sizeof(objects[0]);
    PointLight light = {{-0.5, 0.5, 1.5}, 5.f, 200.0f};
    PointLight light2 = {{0.5, 0.5, 1.5}, 5.f, 200.0f};
    PointLight light3 = {{0, 1, 1.5}, 5.f, 200.0f};
    PointLight lights[] = {light, light2, light3};
    int light_count = sizeof(lights) / sizeof(lights[0]);

    render_scene(&renderer, &camera, objects, object_count, lights, light_count);

    mlx_put_image_to_window(renderer.mlx, renderer.win, renderer.img, 0, 0);
    mlx_loop(renderer.mlx);
    free(renderer.framebuffer);
    return 0;
}