///3.delete overlapped
    std::vector<bool> flag_delete(entity_ids.size(), false);
    for(int i = 0; i < entity_ids.size() - 1; ++i){
        if(flag_delete[i]){
            continue;
        }
        else{
            int refer_traj_begin_id = entities[entity_ids[i]].traj_start_id;
            int refer_traj_end_id = entities[entity_ids[i]].traj_end_id;

            for(int j = i+1; j < entity_ids.size(); ++j){
                if(flag_delete[j]){
                    continue;
                }
                int traj_begin_id = entities[entity_ids[j]].traj_start_id;
                int traj_end_id = entities[entity_ids[j]].traj_end_id;

                /// if overlapped:
                if((refer_traj_begin_id - traj_begin_id) * (refer_traj_end_id - traj_end_id) <= 0){
                    if(fabs(refer_traj_end_id - refer_traj_begin_id) > fabs(traj_end_id - traj_begin_id) ){
                        /// the other is shorter

                        Eigen::Vector3f traj_begin_point = {
                                static_cast<float>(traj_pts_[traj_begin_id]._x_),
                                static_cast<float>(traj_pts_[traj_begin_id]._y_),
                                0.0f
                        };

                        Eigen::Vector3f traj_end_point = {
                                static_cast<float>(traj_pts_[traj_end_id]._x_),
                                static_cast<float>(traj_pts_[traj_end_id]._y_),
                                0.0f
                        };


                        // for shorter one:
                        Eigen::Vector3f end_points_xy_1 =  entities[entity_ids[j]].end_point_1;
                        Eigen::Vector3f end_points_xy_2 =  entities[entity_ids[j]].end_point_2;
                        end_points_xy_1[2] = 0.0f;
                        end_points_xy_2[2] = 0.0f;

                        float d_to_traj = (end_points_xy_1 - traj_begin_point).norm() +
                                          (end_points_xy_2 - traj_end_point).norm();
                        d_to_traj /= 2.0f;

                        // for longer one:
                        float d_to_traj_refer = getMinimumnLengthFromPointToEntityXY(entities[entity_ids[i]], traj_begin_point) +
                                                getMinimumnLengthFromPointToEntityXY(entities[entity_ids[i]], traj_end_point);
                        d_to_traj_refer /= 2.0f;

                        if(fabs(d_to_traj - d_to_traj_refer) < 1.5f && (traj_begin_point - traj_end_point).norm() < 2.5f){
                            flag_delete[j] = true;
                        }


                    }
                    else{
                        /// reference is shorter

                        Eigen::Vector3f traj_begin_point = {
                                static_cast<float>(traj_pts_[refer_traj_begin_id]._x_),
                                static_cast<float>(traj_pts_[refer_traj_begin_id]._y_),
                                0.0f
                        };

                        Eigen::Vector3f traj_end_point = {
                                static_cast<float>(traj_pts_[refer_traj_end_id]._x_),
                                static_cast<float>(traj_pts_[refer_traj_end_id]._y_),
                                0.0f
                        };


                        // for shorter one:
                        Eigen::Vector3f end_points_xy_1 =  entities[entity_ids[i]].end_point_1;
                        Eigen::Vector3f end_points_xy_2 =  entities[entity_ids[i]].end_point_2;
                        end_points_xy_1[2] = 0.0f;
                        end_points_xy_2[2] = 0.0f;

                        float d_to_traj = (end_points_xy_1 - traj_begin_point).norm() +
                                          (end_points_xy_2 - traj_end_point).norm();
                        d_to_traj /= 2.0f;

                        // for longer one:
                        float d_to_traj_refer = getMinimumnLengthFromPointToEntityXY(entities[entity_ids[j]], traj_begin_point) +
                                                getMinimumnLengthFromPointToEntityXY(entities[entity_ids[j]], traj_end_point);
                        d_to_traj_refer /= 2.0f;

                        if(fabs(d_to_traj - d_to_traj_refer) < 1.5f && (traj_begin_point - traj_end_point).norm() < 2.5f){
                            flag_delete[i] = true;
                            break;
                        }
                    }
                }
            }
        }
    }

    std::vector<bool>::iterator it_flag = flag_delete.begin();
    std::vector<int>::iterator it_data = entity_ids.begin();
    while (it_data != entity_ids.end()){
        if(*it_flag){
            it_data = entity_ids.erase(it_data);
        }else{
            ++it_data;
        }

        ++it_flag;
    }

    std::vector<bool>().swap(flag_delete);
    flag_delete.shrink_to_fit();
