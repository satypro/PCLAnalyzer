# PCLAnalyzer
PCLAnalyzer is a tool to conduct some experiment to find the descriptors or a point like  its Neghbours, 
Covaraiance Matrix, Eigen Value , Eigen Matrix and also to define the Tensor a one of the descriptor.

Once, we obtained the descriptors we can do analysis to classify point as a part of surface, line etc.

# TO use the Configuration
    /*Get the config for the Types and show this as the Property in the windows*/
    Configuration* config = search->GetConfig();
    std::map<std::string, std::string>& mapp = config->GetConfig();
    mapp["Radius"] = "1";

    /*for(std::map<std::string, std::string>::iterator it = mapp.begin(); it!=mapp.end(); ++it )
    {
        std::cout<<it->first<<" "<<it->second<<std::endl;
    }
    //config->SetValue("Radius","12");
    */
