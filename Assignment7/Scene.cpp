//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"

void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){                             // 随机选取一个光源面（方法：累加光源面积，直到超过随机数p）
                objects[k]->Sample(pos, pdf);                  // 利用Sample()在光源面中，得到采样所需的 light_pos 和 pdf
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{

    // TO DO Implement Path Tracing Algorithm here
    Intersection interP;
    interP = intersect(ray);
    float EPLISON = 0.0001;

    if (!interP.happened) {
        return Vector3f(0.0f);
    }
    if (interP.m->hasEmission()) {
        return interP.m->getEmission();
    }
    
    Vector3f L_dir;
    // 采样一次随机光源，得到 lightInter_pos 和 pdf 等相关信息
    // interX为光源中，用于采样的点
    Intersection interX;
    float pdf_light = 0.0f;
    sampleLight(interX, pdf_light);

    // Shoot a ray from p to x
    // If the ray is not blocked in the middle, then calculate
    Vector3f px_dir = (interX.coords - interP.coords).normalized();
    float px_dist = (interX.coords - interP.coords).norm();
    Vector3f p_normal = interP.normal;
    Vector3f x_normal = interX.normal;

    Ray px_ray(interP.coords, px_dir);
    Intersection interBlock = intersect(px_ray);
    if (interBlock.distance > px_dist - EPLISON) {
        L_dir += interX.emit * interP.m->eval(-ray.direction, px_dir, p_normal)
            * dotProduct(px_dir, p_normal) * dotProduct(-px_dir, x_normal)
            / (px_dist * px_dist) / pdf_light;
        // 这里的BRDF只写了Diffuse的情况，所以与wi无关，wi方向随意（实际上应当从p点向外）
    }

    Vector3f L_indir = Vector3f(0.0f);
    float p_RR = 0.6f;
    if (get_random_float() > p_RR) {
        return L_dir;
    }
    
    Vector3f wo_dir = -ray.direction;
    Vector3f wi_dir = interP.m->sample(wo_dir, interP.normal).normalized();
    Intersection interQ;
    Ray pq_ray(interP.coords, wi_dir);
    interQ = intersect(pq_ray);

    // If ray r hit a non-emitting object at q, calculate
    if (interQ.happened && (!interQ.m->hasEmission())) {
        L_indir = castRay(pq_ray, depth + 1) * interP.m->eval(wi_dir, wo_dir, interP.normal)
            * dotProduct(wi_dir, interP.normal) / interP.m->pdf(wi_dir, wo_dir, interP.normal)
            / p_RR;
    }
    
    return L_dir + L_indir;
}