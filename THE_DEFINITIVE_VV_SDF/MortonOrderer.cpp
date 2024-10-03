#include "MortonOrderer.h"

size_t MortonOrderer::GetMortonOrder(size_t* inputs, int input_count)
{
    size_t to_return = 0;

    int inp;
    int digit_inp, digit_mort;

    for (inp = 0; inp < input_count; ++inp)
    {
        //64 for the bits in a size_t
        for (digit_inp = 0, digit_mort = inp; digit_mort < 64; ++digit_inp, digit_mort += input_count)
        {
            to_return |= (((inputs[inp] >> digit_inp) & 1) << digit_mort);
        }
    }

    return to_return;
}

void MortonOrderer::GetNormalOrder(size_t* outputs, int output_count, size_t morton_ordered_number)
{
    int outp;
    int digit_outp, digit_mort;

    for (outp = 0; outp < output_count; ++outp)
    {
        outputs[outp] = 0;

        for (digit_outp = 0, digit_mort = outp; digit_mort < 64; ++digit_outp, digit_mort += output_count)
        {
            outputs[outp] |= (((morton_ordered_number >> digit_mort) & 1) << digit_outp);
        }
    }
}
