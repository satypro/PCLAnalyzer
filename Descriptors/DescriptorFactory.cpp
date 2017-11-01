#include "DescriptorFactory.h"
#include "Descriptor.h"

DescriptorBase* DescriptorFactory::_instance;

DescriptorFactory::DescriptorFactory()
{

}

DescriptorBase*  DescriptorFactory::GetDescriptor()
{
    if (DescriptorFactory::_instance == 0)
        DescriptorFactory::_instance = new Descriptor();

    return DescriptorFactory::_instance;
}
