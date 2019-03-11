inline
    void applyMeadianFilterOnZ(std::vector<Eigen::Vector3f>& points, const int window_size_half){
        int start_id = 0;
        int end_id = 0;
        for(int i = 0; i < points.size(); ++i){
            start_id = (i - window_size_half) < 0 ? 0 : (i - window_size_half);
            end_id = (i + window_size_half) > (points.size() - 1) ? (points.size() - 1) : (i + window_size_half);


            // range : [start_id, end_id]
            int vec_size = end_id - start_id + 1;

            if(vec_size > 0){
                std::vector<float> z_vec(vec_size);
                for(int j = 0; j < vec_size; ++j){
                    z_vec[j] = points[start_id + j][2];
                }

                points[i][2] = getMedianValue(z_vec);

                std::vector<float>().swap(z_vec);
                z_vec.shrink_to_fit();
            }

        }
    }
